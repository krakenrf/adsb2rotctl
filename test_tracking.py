#!/usr/bin/env python3
"""Rotator tracking test — simulate an aircraft flyby to evaluate rotator performance."""

import argparse
import json
import math
import os
import socket
import sys
import time

# ── Constants ────────────────────────────────────────────────────────────────

EARTH_RADIUS = 6371000  # meters
FT_TO_M = 0.3048
KT_TO_MS = 0.514444    # knots to m/s
FLYBY_RANGE = 30000     # start/end 30 km from closest approach


# ── AZ/EL math (copied from adsb_rotctl.py) ─────────────────────────────────

def haversine(lat1, lon1, lat2, lon2):
    """Return ground distance in meters between two lat/lon points."""
    lat1, lon1, lat2, lon2 = (math.radians(v) for v in (lat1, lon1, lat2, lon2))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing(lat1, lon1, lat2, lon2):
    """Return initial bearing in degrees (0-360) from point 1 to point 2."""
    lat1, lon1, lat2, lon2 = (math.radians(v) for v in (lat1, lon1, lat2, lon2))
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def elevation(ground_dist, rx_alt, ac_alt):
    """Return elevation angle in degrees accounting for Earth curvature."""
    if ground_dist < 1:
        return 90.0 if ac_alt > rx_alt else 0.0
    theta = ground_dist / EARTH_RADIUS
    r1 = EARTH_RADIUS + rx_alt
    r2 = EARTH_RADIUS + ac_alt
    dx = r2 * math.sin(theta)
    dz = r2 * math.cos(theta) - r1
    return math.degrees(math.atan2(dz, dx))


def calc_az_el(rx_lat, rx_lon, rx_alt, ac_lat, ac_lon, ac_alt):
    """Calculate azimuth and elevation from receiver to aircraft."""
    az = bearing(rx_lat, rx_lon, ac_lat, ac_lon)
    dist = haversine(rx_lat, rx_lon, ac_lat, ac_lon)
    el = elevation(dist, rx_alt, ac_alt)
    el = max(0.0, min(90.0, el))
    return az, el, dist


# ── Geodesic helpers ─────────────────────────────────────────────────────────

def destination_point(lat, lon, bearing_deg, distance_m):
    """Return (lat, lon) after moving distance_m along bearing_deg from (lat, lon)."""
    lat = math.radians(lat)
    lon = math.radians(lon)
    brng = math.radians(bearing_deg)
    d = distance_m / EARTH_RADIUS

    lat2 = math.asin(math.sin(lat) * math.cos(d) + math.cos(lat) * math.sin(d) * math.cos(brng))
    lon2 = lon + math.atan2(
        math.sin(brng) * math.sin(d) * math.cos(lat),
        math.cos(d) - math.sin(lat) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


# ── Rotctld connection (copied from adsb_rotctl.py) ─────────────────────────

class RotctlConnection:
    """Persistent TCP connection to rotctld with auto-reconnect."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        """Establish connection to rotctld."""
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)
        self.sock.connect((self.host, self.port))
        self.sock.setblocking(False)
        print(f"[rotctld] Connected to {self.host}:{self.port}")

    def _drain(self):
        """Non-blocking drain of any pending response data."""
        try:
            while self.sock.recv(256):
                pass
        except BlockingIOError:
            pass
        except OSError:
            pass

    def send_position(self, az, el):
        """Send a position command to rotctld. Reconnects on failure."""
        cmd = f"P {az:.2f} {el:.2f}\n"
        for attempt in range(2):
            try:
                if self.sock is None:
                    self.connect()
                self._drain()
                self.sock.sendall(cmd.encode())
                return True
            except (OSError, BrokenPipeError, ConnectionError):
                self.sock = None
                if attempt == 0:
                    print("[rotctld] Connection lost, reconnecting...")
                    try:
                        self.connect()
                    except OSError as e:
                        print(f"[rotctld] Reconnect failed: {e}")
                        return False
        return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


# ── Receiver location auto-detect ────────────────────────────────────────────

def detect_receiver_location():
    """Try to read receiver lat/lon from local receiver.json."""
    for path in ("/run/readsb", "/run/dump1090-fa", "/run/dump1090"):
        rpath = os.path.join(path, "receiver.json")
        if os.path.isfile(rpath):
            try:
                with open(rpath) as f:
                    data = json.load(f)
                lat, lon = data.get("lat"), data.get("lon")
                if lat is not None and lon is not None:
                    return float(lat), float(lon)
            except Exception:
                pass
    return None, None


# ── Address parsing ──────────────────────────────────────────────────────────

def parse_rotator_address(addr):
    """Parse 'host:port' string into (host, port) tuple."""
    if ":" not in addr:
        print(f"Error: rotator address must be IP:PORT, got '{addr}'", file=sys.stderr)
        sys.exit(1)
    parts = addr.rsplit(":", 1)
    try:
        return parts[0], int(parts[1])
    except ValueError:
        print(f"Error: invalid port in '{addr}'", file=sys.stderr)
        sys.exit(1)


# ── Simulation ───────────────────────────────────────────────────────────────

def run_simulation(rotctl, rx_lat, rx_lon, rx_alt, speed_kt, altitude_ft, distance_km, heading_deg, update_rate):
    """Simulate an aircraft flyby and send position commands to rotctld."""
    speed_ms = speed_kt * KT_TO_MS
    ac_alt = altitude_ft * FT_TO_M
    closest_dist_m = distance_km * 1000

    # Aircraft track is along heading_deg. The closest approach point is
    # offset perpendicular (90° to the right of heading) from the receiver.
    perp_bearing = (heading_deg + 90) % 360
    closest_point = destination_point(rx_lat, rx_lon, perp_bearing, closest_dist_m)

    # Start point: 30 km before closest approach along the reverse heading
    reverse_heading = (heading_deg + 180) % 360
    start_lat, start_lon = destination_point(closest_point[0], closest_point[1], reverse_heading, FLYBY_RANGE)

    total_distance = 2 * FLYBY_RANGE  # 60 km total track
    total_time = total_distance / speed_ms
    dt = 1.0 / update_rate

    print(f"[sim] Aircraft: {speed_kt} kt, {altitude_ft} ft AGL, heading {heading_deg}°")
    print(f"[sim] Closest approach: {distance_km} km from receiver")
    print(f"[sim] Track length: {total_distance / 1000:.0f} km, duration: {total_time:.1f}s")
    print(f"[sim] Update rate: {update_rate} Hz ({dt * 1000:.0f}ms interval)")
    print(f"[sim] Receiver: {rx_lat:.4f}, {rx_lon:.4f}, {rx_alt:.0f}m ASL")
    print()

    prev_az = None
    prev_el = None
    t = 0.0
    start_wall = time.monotonic()

    try:
        while t <= total_time:
            # Current aircraft position along the track
            along_track = speed_ms * t
            ac_lat, ac_lon = destination_point(start_lat, start_lon, heading_deg, along_track)

            # Calculate AZ/EL and slant distance
            az, el, ground_dist = calc_az_el(rx_lat, rx_lon, rx_alt, ac_lat, ac_lon, ac_alt)
            slant_dist = math.sqrt(ground_dist ** 2 + (ac_alt - rx_alt) ** 2)

            # Angular rates
            if prev_az is not None:
                daz = az - prev_az
                # Handle 360/0 wrap
                if daz > 180:
                    daz -= 360
                elif daz < -180:
                    daz += 360
                az_rate = daz / dt
                el_rate = (el - prev_el) / dt
            else:
                az_rate = 0.0
                el_rate = 0.0

            # Send to rotctld
            rotctl.send_position(az, el)

            # Display
            mins, secs = divmod(t, 60)
            print(
                f"\r[{int(mins):02d}:{secs:04.1f}] "
                f"AZ={az:6.2f} EL={el:5.2f} | "
                f"AZ_rate={az_rate:+7.2f}\u00b0/s  EL_rate={el_rate:+7.2f}\u00b0/s | "
                f"dist={slant_dist / 1000:.1f}km",
                end="",
                flush=True,
            )

            prev_az = az
            prev_el = el

            # Sleep until next tick, compensating for processing time
            t += dt
            target_wall = start_wall + t
            sleep_time = target_wall - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    print("\n\n[sim] Done.")


# ── CLI ──────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Rotator tracking test — simulate an aircraft flyby to evaluate rotator performance",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python3 test_tracking.py 192.168.1.50:4533\n"
            "  python3 test_tracking.py localhost:4533 --speed 500 --distance 1 --altitude 3000\n"
            "  python3 test_tracking.py localhost:4533 --speed 250 --altitude 10000 --distance 5\n"
        ),
    )
    parser.add_argument("rotator", help="Rotator address as IP:PORT (e.g., 192.168.1.50:4533)")
    parser.add_argument("--speed", type=float, default=250, metavar="KNOTS", help="Aircraft ground speed in knots (default: 250)")
    parser.add_argument("--altitude", type=float, default=10000, metavar="FEET", help="Aircraft altitude in feet AGL (default: 10000)")
    parser.add_argument("--distance", type=float, default=5, metavar="KM", help="Closest approach distance in km (default: 5)")
    parser.add_argument("--heading", type=float, default=90, metavar="DEG", help="Aircraft track heading in degrees (default: 90)")
    parser.add_argument("--lat", type=float, default=None, help="Receiver latitude (default: from receiver.json)")
    parser.add_argument("--lon", type=float, default=None, help="Receiver longitude (default: from receiver.json)")
    parser.add_argument("--alt", type=float, default=0, help="Receiver altitude in meters ASL (default: 0)")
    parser.add_argument("--update-rate", type=float, default=10, metavar="HZ", help="Commands per second to rotctld (default: 10)")
    return parser.parse_args()


def main():
    args = parse_args()

    rot_host, rot_port = parse_rotator_address(args.rotator)

    # Resolve receiver location
    rx_lat, rx_lon = args.lat, args.lon
    if rx_lat is None or rx_lon is None:
        print("[info] Detecting receiver location from receiver.json...")
        auto_lat, auto_lon = detect_receiver_location()
        if auto_lat is not None:
            rx_lat = rx_lat if rx_lat is not None else auto_lat
            rx_lon = rx_lon if rx_lon is not None else auto_lon
            print(f"[info] Receiver location: {rx_lat:.4f}, {rx_lon:.4f}")
        else:
            print("Error: could not determine receiver location.", file=sys.stderr)
            print("Provide --lat and --lon, or ensure receiver.json is accessible.", file=sys.stderr)
            sys.exit(1)

    # Connect to rotctld
    rotctl = RotctlConnection(rot_host, rot_port)
    try:
        rotctl.connect()
    except OSError as e:
        print(f"Error: could not connect to rotctld at {rot_host}:{rot_port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        run_simulation(
            rotctl,
            rx_lat, rx_lon, args.alt,
            args.speed, args.altitude, args.distance, args.heading,
            args.update_rate,
        )
    finally:
        rotctl.close()


if __name__ == "__main__":
    main()
