# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

Single-file Python script (`adsb_rotctl.py`) that reads ADS-B aircraft data (SBS format on port 30003) and commands a rotctld-compatible antenna rotator to track aircraft. Runs on a Raspberry Pi. No external dependencies — standard library only.

## Running

```bash
# Main script — requires rotctld address, auto-detects receiver location from /run/readsb/receiver.json
python3 adsb_rotctl.py 192.168.86.33:4533

# With explicit receiver location
python3 adsb_rotctl.py IP:PORT --lat X --lon X --alt X --sbs-host HOST --sbs-port 30003

# Lock to specific aircraft
python3 adsb_rotctl.py IP:PORT --icao A1B2C3

# Flyby simulation test (sends position commands to rotctld without needing live ADS-B data)
python3 test_tracking.py localhost:4533 --speed 250 --altitude 10000 --distance 5
```

There are no unit tests, linter config, or build steps.

## Architecture

The main script has these major components running concurrently:

- **SBS reader thread** — connects to SBS TCP port 30003, parses MSG types 1-7, updates the shared `AircraftTable`
- **AircraftTable** — thread-safe dict keyed by ICAO hex, stores per-aircraft state (lat, lon, alt, gs, track, vrate, timestamps)
- **HTTP server** — listens on port 7878 for tar1090 aircraft selection events (`POST /select` with `{"icao": "hex"}`)
- **Main loop (10 Hz)** — selects target aircraft, feeds measurements to the IMM tracker, sends predicted AZ/EL to rotctld
- **RotctlConnection** — persistent TCP connection to rotctld with auto-reconnect

### IMM Tracking Filter

The Interacting Multiple Model (IMM) filter is the core of smooth tracking. It blends two Kalman filter models:

- **CVModel** — 6-state `[e, ve, n, vn, u, vu]` constant-velocity Kalman filter
- **CTModel** — 7-state `[e, ve, n, vn, u, vu, ω]` coordinated-turn Extended Kalman Filter with nonlinear state transition

The IMM cycle in `process_measurement()`: mix states → predict → update → update mode probabilities. Mode probability floor (`MIN_MU=0.001`) prevents model extinction. Markov transition matrix `T=[[0.95,0.05],[0.05,0.95]]`.

All filtering is done in ENU (East-North-Up) Cartesian coordinates, not AZ/EL. The `get_predicted_azel()` method does non-destructive forward prediction blending both models' outputs.

### Matrix Math

Pure-Python list-of-lists matrix operations (up to 7x7) — no numpy. Functions: `mat_zeros`, `mat_eye`, `mat_mul`, `mat_add`, `mat_transpose`, `mat_inv`, `mat_scale`, etc.

## Key Constraints

- Must remain pure Python (no pip packages) for easy deployment on Raspberry Pi
- Flat-earth approximation in `latlon_to_enu` is intentional — adequate for <200km ranges
- Process noise tuning: `ACCEL_NOISE=0.5 m/s²`, `OMEGA_NOISE=0.03 rad/s²`
- `pos_time`/`vel_time` timestamps on aircraft entries distinguish new measurements from stale data
