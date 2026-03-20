#!/usr/bin/env python3
# adsb_rotctl — ADS-B to Rotator Controller
# Copyright (C) 2026
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""ADS-B to Rotator Controller — points an antenna rotator at aircraft."""

import argparse
import json
import math
import os
import socket
import sys
import threading
import time
import urllib.request
import urllib.error
from http.server import HTTPServer, BaseHTTPRequestHandler

# ── Globals ──────────────────────────────────────────────────────────────────

target_icao = None        # Currently selected ICAO hex (None = track closest)
target_lock = threading.Lock()

# QNH barometric correction: metres to add to pressure altitude for true altitude.
# Updated periodically from Open-Meteo.  Positive when QNH > 1013.25 hPa.
STANDARD_QNH = 1013.25       # hPa — standard sea-level pressure
HPFT = 30.0                  # feet per hPa (ISA approximation)
qnh_correction_m = 0.0       # global, updated by qnh_updater thread
qnh_lock = threading.Lock()

# ── AZ/EL math ──────────────────────────────────────────────────────────────

EARTH_RADIUS = 6371000  # meters
FT_TO_M = 0.3048


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
    """Return elevation angle in degrees accounting for Earth curvature.

    Uses spherical geometry: given ground distance along the surface,
    computes the true look angle from the observer to the aircraft.
    """
    if ground_dist < 1:
        return 90.0 if ac_alt > rx_alt else 0.0
    theta = ground_dist / EARTH_RADIUS                       # central angle
    r1 = EARTH_RADIUS + rx_alt
    r2 = EARTH_RADIUS + ac_alt
    # Target position in observer's local tangent frame
    dx = r2 * math.sin(theta)                                # horizontal
    dz = r2 * math.cos(theta) - r1                           # vertical
    return math.degrees(math.atan2(dz, dx))


def calc_az_el(rx_lat, rx_lon, rx_alt, ac_lat, ac_lon, ac_alt):
    """Calculate azimuth and elevation from receiver to aircraft."""
    az = bearing(rx_lat, rx_lon, ac_lat, ac_lon)
    dist = haversine(rx_lat, rx_lon, ac_lat, ac_lon)
    el = elevation(dist, rx_alt, ac_alt)
    el = max(0.0, min(90.0, el))
    return az, el

# ── ENU coordinate conversions ──────────────────────────────────────────────

KT_TO_MS = 0.514444
FPM_TO_MS = 0.00508


def latlon_to_enu(lat, lon, alt, rx_lat, rx_lon, rx_alt):
    """Convert lat/lon/alt to East-North-Up meters relative to receiver.

    Uses flat-earth for east/north (adequate <200 km) with spherical
    geometry correction for the up component to account for Earth curvature.
    """
    dlat = math.radians(lat - rx_lat)
    dlon = math.radians(lon - rx_lon)
    east = dlon * math.cos(math.radians(rx_lat)) * EARTH_RADIUS
    north = dlat * EARTH_RADIUS
    # Spherical correction: at ground distance d the surface curves away
    # by R - R*cos(d/R) ≈ d²/(2R).  Compute the target position in the
    # observer's local tangent plane using exact trig.
    ground_dist_sq = east * east + north * north
    theta = math.sqrt(ground_dist_sq) / EARTH_RADIUS          # central angle
    r2 = EARTH_RADIUS + alt
    r1 = EARTH_RADIUS + rx_alt
    up = r2 * math.cos(theta) - r1
    return east, north, up


def velocity_to_enu(gs_kt, track_deg, vrate_fpm):
    """Convert ADS-B ground speed/track/vertical rate to ENU velocity (m/s)."""
    gs = gs_kt * KT_TO_MS
    track_rad = math.radians(track_deg)
    ve = gs * math.sin(track_rad)
    vn = gs * math.cos(track_rad)
    vu = vrate_fpm * FPM_TO_MS
    return ve, vn, vu


def enu_to_azel(east, north, up):
    """Convert ENU position to azimuth and elevation (degrees)."""
    az = math.degrees(math.atan2(east, north)) % 360
    ground_dist = math.sqrt(east * east + north * north)
    if ground_dist < 1:
        el = 90.0 if up > 0 else 0.0
    else:
        el = math.degrees(math.atan2(up, ground_dist))
    el = max(0.0, min(90.0, el))
    return az, el

# ── Matrix math utilities (list-of-lists) ───────────────────────────────────


def mat_zeros(n, m=None):
    if m is None:
        m = n
    return [[0.0] * m for _ in range(n)]


def mat_identity(n):
    M = mat_zeros(n)
    for i in range(n):
        M[i][i] = 1.0
    return M


def mat_add(A, B):
    n = len(A)
    m = len(A[0])
    return [[A[i][j] + B[i][j] for j in range(m)] for i in range(n)]


def mat_sub(A, B):
    n = len(A)
    m = len(A[0])
    return [[A[i][j] - B[i][j] for j in range(m)] for i in range(n)]


def mat_mul(A, B):
    n = len(A)
    m = len(B[0])
    p = len(B)
    C = mat_zeros(n, m)
    for i in range(n):
        for j in range(m):
            s = 0.0
            for k in range(p):
                s += A[i][k] * B[k][j]
            C[i][j] = s
    return C


def mat_transpose(A):
    n = len(A)
    m = len(A[0])
    return [[A[i][j] for i in range(n)] for j in range(m)]


def mat_inverse(A):
    """Gauss-Jordan inverse for small matrices."""
    n = len(A)
    M = [A[i][:] + [1.0 if j == i else 0.0 for j in range(n)] for i in range(n)]
    for col in range(n):
        max_row = col
        for row in range(col + 1, n):
            if abs(M[row][col]) > abs(M[max_row][col]):
                max_row = row
        M[col], M[max_row] = M[max_row], M[col]
        pivot = M[col][col]
        if abs(pivot) < 1e-12:
            return mat_identity(n)
        inv_pivot = 1.0 / pivot
        for j in range(2 * n):
            M[col][j] *= inv_pivot
        for row in range(n):
            if row == col:
                continue
            factor = M[row][col]
            for j in range(2 * n):
                M[row][j] -= factor * M[col][j]
    return [M[i][n:] for i in range(n)]


def vec_to_col(v):
    """Convert a list to a column matrix (n x 1)."""
    return [[x] for x in v]


def col_to_vec(C):
    """Convert a column matrix (n x 1) to a list."""
    return [C[i][0] for i in range(len(C))]


def mat_scale(A, s):
    """Multiply every element of matrix A by scalar s."""
    return [[A[i][j] * s for j in range(len(A[0]))] for i in range(len(A))]


def mat_determinant(A):
    """NxN determinant via row reduction (partial pivoting)."""
    n = len(A)
    M = [row[:] for row in A]
    sign = 1.0
    for col in range(n):
        max_row = col
        for row in range(col + 1, n):
            if abs(M[row][col]) > abs(M[max_row][col]):
                max_row = row
        if max_row != col:
            M[col], M[max_row] = M[max_row], M[col]
            sign *= -1.0
        pivot = M[col][col]
        if abs(pivot) < 1e-30:
            return 0.0
        for row in range(col + 1, n):
            factor = M[row][col] / pivot
            for j in range(col + 1, n):
                M[row][j] -= factor * M[col][j]
    det = sign
    for i in range(n):
        det *= M[i][i]
    return det

# ── Kalman models & IMM tracker ─────────────────────────────────────────────

ACCEL_NOISE = 0.5        # m/s^2 — process noise acceleration SD
POS_NOISE_M = 50.0       # m — ADS-B position measurement SD
ALT_NOISE_M = 30.0       # m — barometric altitude SD
VEL_NOISE_MS = 5.0       # m/s — ground speed SD
VRATE_NOISE_MS = 3.0     # m/s — vertical rate SD
STALE_THRESHOLD = 10.0   # s — reinitialize if no data for this long
OMEGA_NOISE = 0.03       # rad/s^2 — turn rate random walk SD

LOG_2PI = math.log(2.0 * math.pi)


def _kalman_log_likelihood(z, H, R, x, P, n_state):
    """Compute Kalman update and return (updated x, updated P, log-likelihood)."""
    x_col = vec_to_col(x)
    z_col = vec_to_col(z)
    y_col = mat_sub(z_col, mat_mul(H, x_col))
    y = col_to_vec(y_col)
    HP = mat_mul(H, P)
    S = mat_add(mat_mul(HP, mat_transpose(H)), R)
    S_inv = mat_inverse(S)
    PHt = mat_mul(P, mat_transpose(H))
    K = mat_mul(PHt, S_inv)
    Ky = col_to_vec(mat_mul(K, y_col))
    x_new = [x[i] + Ky[i] for i in range(n_state)]
    KH = mat_mul(K, H)
    IKH = mat_sub(mat_identity(n_state), KH)
    P_new = mat_mul(IKH, P)

    # Log-likelihood: -0.5 * (m*log(2π) + log|S| + y'*S^-1*y)
    m = len(z)
    det_S = max(mat_determinant(S), 1e-30)
    mahal_mat = mat_mul(mat_mul(mat_transpose(y_col), S_inv), y_col)
    mahal = mahal_mat[0][0]
    ll = -0.5 * (m * LOG_2PI + math.log(det_S) + mahal)

    return x_new, P_new, ll


class CVModel:
    """6-state constant-velocity Kalman filter model.

    State: [east, v_east, north, v_north, up, v_up]
    """

    def __init__(self):
        self.x = [0.0] * 6
        self.P = mat_identity(6)

    def set_state(self, x, P):
        self.x = x[:]
        self.P = [row[:] for row in P]

    def make_F(self, dt):
        """6x6 state transition matrix — constant velocity."""
        F = mat_identity(6)
        F[0][1] = dt
        F[2][3] = dt
        F[4][5] = dt
        return F

    def make_Q(self, dt):
        """6x6 process noise — continuous white noise acceleration model."""
        q = ACCEL_NOISE ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        Q = mat_zeros(6)
        for i in range(3):
            p = 2 * i
            v = p + 1
            Q[p][p] = q * dt3 / 3.0
            Q[p][v] = q * dt2 / 2.0
            Q[v][p] = q * dt2 / 2.0
            Q[v][v] = q * dt
        return Q

    def predict(self, dt):
        """Advance state and covariance by dt (destructive)."""
        if dt <= 0:
            return
        F = self.make_F(dt)
        Q = self.make_Q(dt)
        x_col = vec_to_col(self.x)
        self.x = col_to_vec(mat_mul(F, x_col))
        FP = mat_mul(F, self.P)
        self.P = mat_add(mat_mul(FP, mat_transpose(F)), Q)

    def predict_state(self, dt):
        """Non-destructive prediction, returns predicted state list."""
        if dt <= 0:
            return self.x[:]
        F = self.make_F(dt)
        x_col = vec_to_col(self.x)
        return col_to_vec(mat_mul(F, x_col))

    def update(self, z, H, R):
        """Kalman measurement update. Returns log-likelihood."""
        self.x, self.P, ll = _kalman_log_likelihood(z, H, R, self.x, self.P, 6)
        return ll


class CTModel:
    """7-state coordinated-turn EKF model.

    State: [east, v_east, north, v_north, up, v_up, omega]
    omega is turn rate in rad/s.
    """

    def __init__(self):
        self.x = [0.0] * 7
        self.P = mat_identity(7)

    def set_state(self, x, P):
        self.x = x[:]
        self.P = [row[:] for row in P]

    @staticmethod
    def _ct_predict_state(x, dt):
        """Apply nonlinear coordinated-turn transition."""
        e, ve, n, vn, u, vu, w = x
        if abs(w) > 1e-6:
            sw = math.sin(w * dt)
            cw = math.cos(w * dt)
            sw_w = sw / w
            cw1_w = (1.0 - cw) / w
            e_new = e + sw_w * ve + cw1_w * vn
            ve_new = cw * ve + sw * vn
            n_new = n - cw1_w * ve + sw_w * vn
            vn_new = -sw * ve + cw * vn
        else:
            # Degenerate to CV when omega ≈ 0
            e_new = e + ve * dt
            ve_new = ve
            n_new = n + vn * dt
            vn_new = vn
        u_new = u + vu * dt
        vu_new = vu
        w_new = w
        return [e_new, ve_new, n_new, vn_new, u_new, vu_new, w_new]

    @staticmethod
    def _ct_jacobian(x, dt):
        """7x7 Jacobian of the CT transition function."""
        ve, vn, w = x[1], x[3], x[6]
        F = mat_identity(7)

        if abs(w) > 1e-6:
            sw = math.sin(w * dt)
            cw = math.cos(w * dt)
            sw_w = sw / w
            cw1_w = (1.0 - cw) / w
            w2 = w * w

            # ∂e/∂(ve, vn)
            F[0][1] = sw_w
            F[0][3] = cw1_w
            # ∂ve/∂(ve, vn)
            F[1][1] = cw
            F[1][3] = sw
            # ∂n/∂(ve, vn)
            F[2][1] = -cw1_w
            F[2][3] = sw_w
            # ∂vn/∂(ve, vn)
            F[3][1] = -sw
            F[3][3] = cw

            # ∂/∂ω (partial derivatives w.r.t. turn rate)
            # ∂e/∂ω
            F[0][6] = (cw * dt / w - sw / w2) * ve + (sw * dt / w - (1.0 - cw) / w2) * vn
            # ∂ve/∂ω
            F[1][6] = -sw * dt * ve + cw * dt * vn
            # ∂n/∂ω
            F[2][6] = -(sw * dt / w - (1.0 - cw) / w2) * ve + (cw * dt / w - sw / w2) * vn
            # ∂vn/∂ω
            F[3][6] = -cw * dt * ve - sw * dt * vn
        else:
            F[0][1] = dt
            F[2][3] = dt

        # Vertical channel (always CV)
        F[4][5] = dt

        return F

    def make_Q(self, dt):
        """7x7 process noise — CV noise for position/velocity + random walk for omega."""
        q = ACCEL_NOISE ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        Q = mat_zeros(7)
        for i in range(3):
            p = 2 * i
            v = p + 1
            Q[p][p] = q * dt3 / 3.0
            Q[p][v] = q * dt2 / 2.0
            Q[v][p] = q * dt2 / 2.0
            Q[v][v] = q * dt
        Q[6][6] = OMEGA_NOISE ** 2 * dt
        return Q

    def predict(self, dt):
        """EKF predict: nonlinear state transition, Jacobian for covariance."""
        if dt <= 0:
            return
        self.x = self._ct_predict_state(self.x, dt)
        F = self._ct_jacobian(self.x, dt)
        Q = self.make_Q(dt)
        FP = mat_mul(F, self.P)
        self.P = mat_add(mat_mul(FP, mat_transpose(F)), Q)

    def predict_state(self, dt):
        """Non-destructive prediction, returns predicted state list."""
        if dt <= 0:
            return self.x[:]
        return self._ct_predict_state(self.x, dt)

    def update(self, z, H, R):
        """EKF measurement update (linear observation). Returns log-likelihood."""
        self.x, self.P, ll = _kalman_log_likelihood(z, H, R, self.x, self.P, 7)
        return ll


class IMMTracker:
    """Interacting Multiple Model tracker — blends CV and CT Kalman filters.

    Same public API as the former KalmanTracker:
        __init__, switch_target, process_measurement, get_predicted_azel
    """

    # Markov transition probabilities: T[i][j] = P(switch from model i to j)
    T = [[0.95, 0.05],
         [0.05, 0.95]]
    MIN_MU = 0.001  # mode probability floor

    def __init__(self, rx_lat, rx_lon, rx_alt):
        self.rx_lat = rx_lat
        self.rx_lon = rx_lon
        self.rx_alt = rx_alt
        self.models = [CVModel(), CTModel()]
        self.mu = [0.9, 0.1]  # initial mode probabilities (favor CV)
        self.last_time = None
        self.current_icao = None
        self._last_pos_time = None
        self._last_vel_time = None
        self.initialized = False

    # ── State mapping between CV (6-state) and common 7-state space ──

    @staticmethod
    def _cv_to_common(x, P):
        """Expand CV 6-state to 7-state common space (append omega=0)."""
        x7 = x[:] + [0.0]
        P7 = mat_zeros(7)
        for i in range(6):
            for j in range(6):
                P7[i][j] = P[i][j]
        P7[6][6] = 0.01  # small initial omega variance
        return x7, P7

    @staticmethod
    def _common_to_cv(x7, P7):
        """Shrink 7-state to CV 6-state (drop omega row/col)."""
        x6 = x7[:6]
        P6 = mat_zeros(6)
        for i in range(6):
            for j in range(6):
                P6[i][j] = P7[i][j]
        return x6, P6

    @staticmethod
    def _ct_to_common(x, P):
        """CT is already 7-state — identity mapping."""
        return x[:], [row[:] for row in P]

    @staticmethod
    def _common_to_ct(x7, P7):
        """Identity mapping."""
        return x7[:], [row[:] for row in P7]

    def _to_common(self, idx):
        if idx == 0:
            return self._cv_to_common(self.models[0].x, self.models[0].P)
        else:
            return self._ct_to_common(self.models[1].x, self.models[1].P)

    def _from_common(self, idx, x7, P7):
        if idx == 0:
            return self._common_to_cv(x7, P7)
        else:
            return self._common_to_ct(x7, P7)

    # ── IMM mixing step ──

    def _mix(self):
        """Compute mixing probabilities and blend model states in common space."""
        n_models = 2
        # Predicted mode probabilities: c_j = sum_i mu_i * T_ij
        c = [0.0] * n_models
        for j in range(n_models):
            for i in range(n_models):
                c[j] += self.mu[i] * self.T[i][j]
            c[j] = max(c[j], 1e-30)

        # Mixing weights: mu_ij = mu_i * T_ij / c_j
        mix_w = [[0.0] * n_models for _ in range(n_models)]
        for j in range(n_models):
            for i in range(n_models):
                mix_w[i][j] = self.mu[i] * self.T[i][j] / c[j]

        # Get all models in common 7-state space
        commons = [self._to_common(i) for i in range(n_models)]

        # For each target model j, compute mixed state and covariance
        for j in range(n_models):
            # Mixed mean: x_mix_j = sum_i mix_w[i][j] * x_common_i
            x_mix = [0.0] * 7
            for i in range(n_models):
                xi, _ = commons[i]
                for k in range(7):
                    x_mix[k] += mix_w[i][j] * xi[k]

            # Mixed covariance: P_mix_j = sum_i mix_w[i][j] * (P_i + dx*dx')
            P_mix = mat_zeros(7)
            for i in range(n_models):
                xi, Pi = commons[i]
                dx = [xi[k] - x_mix[k] for k in range(7)]
                dx_col = vec_to_col(dx)
                dx_outer = mat_mul(dx_col, mat_transpose(dx_col))
                contrib = mat_add(Pi, dx_outer)
                for r in range(7):
                    for s in range(7):
                        P_mix[r][s] += mix_w[i][j] * contrib[r][s]

            # Map back to model's native state space and set
            x_j, P_j = self._from_common(j, x_mix, P_mix)
            self.models[j].set_state(x_j, P_j)

        return c

    # ── Mode probability update ──

    def _update_mu(self, log_likelihoods, c):
        """Update mode probabilities from log-likelihoods (log-space for stability)."""
        n = len(log_likelihoods)
        # log(mu_j) = log(L_j) + log(c_j), then normalize
        log_mu = [log_likelihoods[j] + math.log(max(c[j], 1e-30)) for j in range(n)]
        max_log = max(log_mu)
        # exp and normalize
        exp_mu = [math.exp(log_mu[j] - max_log) for j in range(n)]
        total = sum(exp_mu)
        if total < 1e-30:
            total = 1e-30
        self.mu = [exp_mu[j] / total for j in range(n)]
        # Floor
        for j in range(n):
            if self.mu[j] < self.MIN_MU:
                self.mu[j] = self.MIN_MU
        # Re-normalize after floor
        total = sum(self.mu)
        self.mu = [self.mu[j] / total for j in range(n)]

    # ── Public API (same as former KalmanTracker) ──

    def initialize(self, ac, now):
        """Set state from first measurement."""
        e, n, u = latlon_to_enu(ac["lat"], ac["lon"], ac["alt"],
                                self.rx_lat, self.rx_lon, self.rx_alt)
        if ac.get("gs") is not None and ac.get("track") is not None:
            vr = ac["vrate"] if ac.get("vrate") is not None else 0.0
            ve, vn, vu = velocity_to_enu(ac["gs"], ac["track"], vr)
        else:
            ve, vn, vu = 0.0, 0.0, 0.0

        x6 = [e, ve, n, vn, u, vu]
        P6 = mat_zeros(6)
        P6[0][0] = POS_NOISE_M ** 2
        P6[2][2] = POS_NOISE_M ** 2
        P6[4][4] = ALT_NOISE_M ** 2
        if ac.get("gs") is not None:
            P6[1][1] = VEL_NOISE_MS ** 2
            P6[3][3] = VEL_NOISE_MS ** 2
        else:
            P6[1][1] = 100.0 ** 2
            P6[3][3] = 100.0 ** 2
        if ac.get("vrate") is not None:
            P6[5][5] = VRATE_NOISE_MS ** 2
        else:
            P6[5][5] = 50.0 ** 2

        # Initialize CV model directly
        self.models[0].set_state(x6, P6)

        # Initialize CT model (7-state: append omega=0)
        x7 = x6 + [0.0]
        P7 = mat_zeros(7)
        for i in range(6):
            for j in range(6):
                P7[i][j] = P6[i][j]
        P7[6][6] = 0.01
        self.models[1].set_state(x7, P7)

        self.mu = [0.9, 0.1]
        self.last_time = now
        self._last_pos_time = ac.get("pos_time")
        self._last_vel_time = ac.get("vel_time")
        self.initialized = True

    def process_measurement(self, ac, now):
        """Incorporate new position/velocity data if available."""
        if ac is None:
            return
        if ac["lat"] is None or ac["lon"] is None or ac["alt"] is None:
            return

        if self.initialized and self.last_time is not None:
            if now - self.last_time > STALE_THRESHOLD:
                self.initialized = False

        if not self.initialized:
            self.initialize(ac, now)
            return

        new_pos = (ac.get("pos_time") is not None and
                   ac["pos_time"] != self._last_pos_time)
        new_vel = (ac.get("vel_time") is not None and
                   ac["vel_time"] != self._last_vel_time)

        if not new_pos and not new_vel:
            return

        dt = now - self.last_time
        self.last_time = now

        # ── IMM cycle: mix → predict → update → mode update ──

        # 1. Mix
        c = self._mix()

        # 2. Predict
        if dt > 0:
            for model in self.models:
                model.predict(dt)

        # 3. Update (collect log-likelihoods from each model)
        log_liks = [0.0, 0.0]

        if new_pos:
            e, n_enu, u = latlon_to_enu(ac["lat"], ac["lon"], ac["alt"],
                                        self.rx_lat, self.rx_lon, self.rx_alt)
            z = [e, n_enu, u]
            R = mat_zeros(3)
            R[0][0] = POS_NOISE_M ** 2
            R[1][1] = POS_NOISE_M ** 2
            R[2][2] = ALT_NOISE_M ** 2

            # CV: 3x6 observation
            H_cv = mat_zeros(3, 6)
            H_cv[0][0] = 1.0
            H_cv[1][2] = 1.0
            H_cv[2][4] = 1.0
            log_liks[0] += self.models[0].update(z, H_cv, R)

            # CT: 3x7 observation
            H_ct = mat_zeros(3, 7)
            H_ct[0][0] = 1.0
            H_ct[1][2] = 1.0
            H_ct[2][4] = 1.0
            log_liks[1] += self.models[1].update(z, H_ct, R)

            self._last_pos_time = ac["pos_time"]

        if new_vel and ac.get("gs") is not None and ac.get("track") is not None:
            vr = ac["vrate"] if ac.get("vrate") is not None else 0.0
            ve, vn_enu, vu = velocity_to_enu(ac["gs"], ac["track"], vr)
            z = [ve, vn_enu, vu]
            R = mat_zeros(3)
            R[0][0] = VEL_NOISE_MS ** 2
            R[1][1] = VEL_NOISE_MS ** 2
            R[2][2] = VRATE_NOISE_MS ** 2

            # CV: 3x6 observation
            H_cv = mat_zeros(3, 6)
            H_cv[0][1] = 1.0
            H_cv[1][3] = 1.0
            H_cv[2][5] = 1.0
            log_liks[0] += self.models[0].update(z, H_cv, R)

            # CT: 3x7 observation
            H_ct = mat_zeros(3, 7)
            H_ct[0][1] = 1.0
            H_ct[1][3] = 1.0
            H_ct[2][5] = 1.0
            log_liks[1] += self.models[1].update(z, H_ct, R)

            self._last_vel_time = ac["vel_time"]

        # 4. Mode probability update
        self._update_mu(log_liks, c)

    def get_predicted_azel(self, now):
        """Non-destructive weighted blend of both models' predicted ENU → AZ/EL."""
        if not self.initialized:
            return None, None
        dt = now - self.last_time

        # Get predicted state from each model, blend in ENU
        e_blend, n_blend, u_blend = 0.0, 0.0, 0.0
        for idx, model in enumerate(self.models):
            x_pred = model.predict_state(dt)
            e_blend += self.mu[idx] * x_pred[0]
            n_blend += self.mu[idx] * x_pred[2]
            u_blend += self.mu[idx] * x_pred[4]

        return enu_to_azel(e_blend, n_blend, u_blend)

    def switch_target(self, icao):
        """Reset filter if tracking a different aircraft."""
        if icao != self.current_icao:
            self.current_icao = icao
            self.initialized = False
            self.last_time = None
            self._last_pos_time = None
            self._last_vel_time = None

# ── Aircraft state table ────────────────────────────────────────────────────


class AircraftTable:
    """Thread-safe table of aircraft state, populated from SBS messages."""

    STALE_SECONDS = 60  # drop aircraft not seen for this long

    def __init__(self):
        self.aircraft = {}  # icao -> {lat, lon, alt, flight, last_seen, gs, track, vrate, ...}
        self.lock = threading.Lock()

    def update_from_sbs(self, fields):
        """Update aircraft state from a parsed SBS/BaseStation CSV line."""
        # SBS format: MSG,type,sess,aircraft,icao,sess,date,time,date,time,...
        if len(fields) < 11 or fields[0] != "MSG":
            return None
        msg_type = fields[1]
        icao = fields[4].lower().strip()
        if not icao:
            return None

        with self.lock:
            ac = self.aircraft.get(icao)
            if ac is None:
                ac = {"lat": None, "lon": None, "alt": None, "flight": None,
                      "last_seen": 0, "gs": None, "track": None, "vrate": None,
                      "pos_time": None, "vel_time": None}
                self.aircraft[icao] = ac

            ac["last_seen"] = time.monotonic()
            updated_position = False
            updated_velocity = False

            # MSG,1: callsign/flight
            if msg_type == "1" and len(fields) > 10:
                cs = fields[10].strip()
                if cs:
                    ac["flight"] = cs

            # MSG,2: ground speed, track, lat, lon (surface position)
            elif msg_type == "2" and len(fields) > 15:
                lat, lon = fields[14], fields[15]
                if lat and lon:
                    try:
                        ac["lat"] = float(lat)
                        ac["lon"] = float(lon)
                        ac["pos_time"] = time.monotonic()
                        updated_position = True
                    except ValueError:
                        pass
                alt = fields[11]
                if alt:
                    try:
                        ac["alt"] = float(alt) * FT_TO_M + qnh_correction_m
                    except ValueError:
                        pass
                gs_str, track_str = fields[12], fields[13]
                if gs_str and track_str:
                    try:
                        ac["gs"] = float(gs_str)
                        ac["track"] = float(track_str)
                        ac["vel_time"] = time.monotonic()
                        updated_velocity = True
                    except ValueError:
                        pass

            # MSG,3: airborne position (lat, lon, alt)
            elif msg_type == "3" and len(fields) > 15:
                alt = fields[11]
                if alt:
                    try:
                        ac["alt"] = float(alt) * FT_TO_M + qnh_correction_m
                    except ValueError:
                        pass
                lat, lon = fields[14], fields[15]
                if lat and lon:
                    try:
                        ac["lat"] = float(lat)
                        ac["lon"] = float(lon)
                        ac["pos_time"] = time.monotonic()
                        updated_position = True
                    except ValueError:
                        pass
                gs_str, track_str = fields[12], fields[13]
                if gs_str and track_str:
                    try:
                        ac["gs"] = float(gs_str)
                        ac["track"] = float(track_str)
                        ac["vel_time"] = time.monotonic()
                        updated_velocity = True
                    except ValueError:
                        pass
                if len(fields) > 16 and fields[16]:
                    try:
                        ac["vrate"] = float(fields[16])
                    except ValueError:
                        pass

            # MSG,4: airborne velocity
            elif msg_type == "4" and len(fields) > 16:
                gs_str, track_str = fields[12], fields[13]
                if gs_str and track_str:
                    try:
                        ac["gs"] = float(gs_str)
                        ac["track"] = float(track_str)
                        ac["vel_time"] = time.monotonic()
                        updated_velocity = True
                    except ValueError:
                        pass
                if fields[16]:
                    try:
                        ac["vrate"] = float(fields[16])
                    except ValueError:
                        pass

            # MSG,5: altitude change
            elif msg_type == "5" and len(fields) > 11:
                alt = fields[11]
                if alt:
                    try:
                        ac["alt"] = float(alt) * FT_TO_M + qnh_correction_m
                    except ValueError:
                        pass

            # MSG,6: squawk
            # MSG,7: air-to-air (altitude only)
            elif msg_type == "7" and len(fields) > 11:
                alt = fields[11]
                if alt:
                    try:
                        ac["alt"] = float(alt) * FT_TO_M + qnh_correction_m
                    except ValueError:
                        pass

            if updated_position or updated_velocity:
                return icao
        return None

    def prune_stale(self):
        """Remove aircraft not seen recently."""
        cutoff = time.monotonic() - self.STALE_SECONDS
        with self.lock:
            stale = [k for k, v in self.aircraft.items() if v["last_seen"] < cutoff]
            for k in stale:
                del self.aircraft[k]

    def get_aircraft(self, icao):
        """Get a single aircraft by ICAO if it has full position data."""
        with self.lock:
            ac = self.aircraft.get(icao)
            if ac and ac["lat"] is not None and ac["lon"] is not None and ac["alt"] is not None:
                return dict(ac)
        return None

    def get_closest(self, rx_lat, rx_lon):
        """Get the closest aircraft with full position data."""
        best = None
        best_dist = float("inf")
        with self.lock:
            for icao, ac in self.aircraft.items():
                if ac["lat"] is None or ac["lon"] is None or ac["alt"] is None:
                    continue
                dist = haversine(rx_lat, rx_lon, ac["lat"], ac["lon"])
                if dist < best_dist:
                    best_dist = dist
                    best = (icao, dict(ac))
        return best

# ── SBS stream reader ───────────────────────────────────────────────────────


def sbs_reader(host, port, table, on_position_update):
    """Connect to SBS port and feed messages into the aircraft table.
    Calls on_position_update(icao) when a position/velocity message is received.
    Runs forever, auto-reconnects."""
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((host, port))
            print(f"[sbs] Connected to {host}:{port}")
            sock.settimeout(None)
            buf = ""
            while True:
                data = sock.recv(4096)
                if not data:
                    break
                buf += data.decode("ascii", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    fields = line.split(",")
                    updated_icao = table.update_from_sbs(fields)
                    if updated_icao:
                        on_position_update(updated_icao)
        except (OSError, ConnectionError) as e:
            print(f"[sbs] Connection error: {e}, reconnecting in 1s...")
        finally:
            try:
                sock.close()
            except Exception:
                pass
        time.sleep(1)

# ── Rotctld TCP connection ──────────────────────────────────────────────────


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

# ── HTTP server for tar1090 selection events ─────────────────────────────────


class SelectionHandler(BaseHTTPRequestHandler):
    """Handles aircraft selection events from the tar1090 JS hook."""

    def do_POST(self):
        global target_icao
        if self.path == "/select":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length) if length else b""
                data = json.loads(body) if body else {}
                icao = data.get("icao")
                with target_lock:
                    target_icao = icao.lower().strip() if icao else None
                if icao:
                    print(f"[tar1090] Selected aircraft: {icao}")
                else:
                    print("[tar1090] Deselected — tracking closest aircraft")
                self.send_response(200)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(b'{"ok":true}')
            except Exception as e:
                print(f"[tar1090] Bad request: {e}")
                self.send_response(400)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def do_GET(self):
        """GET /select returns current target (useful for debugging)."""
        if self.path == "/select":
            with target_lock:
                icao = target_icao
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps({"icao": icao}).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def do_OPTIONS(self):
        """Handle CORS preflight."""
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress default logging

# ── Helpers ──────────────────────────────────────────────────────────────────


def fetch_json(url):
    """Fetch and parse JSON from a URL."""
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as resp:
        return json.loads(resp.read().decode())


def load_json_file(path):
    """Read and parse a JSON file from disk."""
    with open(path, "r") as f:
        return json.load(f)


def detect_json_dir():
    """Auto-detect the readsb/dump1090 JSON directory."""
    for path in ("/run/readsb", "/run/dump1090-fa", "/run/dump1090"):
        if os.path.isfile(os.path.join(path, "receiver.json")):
            return path
    return None


def get_receiver_location(json_dir, base_url):
    """Get receiver lat/lon from receiver.json (file or HTTP)."""
    if json_dir:
        rpath = os.path.join(json_dir, "receiver.json")
        try:
            data = load_json_file(rpath)
            lat, lon = data.get("lat"), data.get("lon")
            if lat is not None and lon is not None:
                return float(lat), float(lon)
        except Exception as e:
            print(f"[warning] Could not read {rpath}: {e}")

    url = base_url.rstrip("/") + "/data/receiver.json"
    try:
        data = fetch_json(url)
        lat, lon = data.get("lat"), data.get("lon")
        if lat is not None and lon is not None:
            return float(lat), float(lon)
    except Exception as e:
        print(f"[warning] Could not fetch receiver.json via HTTP: {e}")
    return None, None


# ── Open-Meteo helpers (elevation + QNH) ─────────────────────────────────────

OPEN_METEO_ELEVATION_URL = "https://api.open-meteo.com/v1/elevation"
OPEN_METEO_FORECAST_URL = "https://api.open-meteo.com/v1/forecast"
QNH_UPDATE_INTERVAL = 600  # seconds between QNH refreshes


def fetch_elevation(lat, lon):
    """Fetch ground elevation (m ASL) from Open-Meteo for a given lat/lon."""
    url = f"{OPEN_METEO_ELEVATION_URL}?latitude={lat}&longitude={lon}"
    data = fetch_json(url)
    elev = data.get("elevation")
    if isinstance(elev, list) and elev:
        return float(elev[0])
    if isinstance(elev, (int, float)):
        return float(elev)
    return None


def fetch_qnh(lat, lon):
    """Fetch current sea-level pressure (QNH) in hPa from Open-Meteo."""
    url = f"{OPEN_METEO_FORECAST_URL}?latitude={lat}&longitude={lon}&current=pressure_msl"
    data = fetch_json(url)
    current = data.get("current", {})
    qnh = current.get("pressure_msl")
    if qnh is not None:
        return float(qnh)
    return None


def qnh_updater(lat, lon):
    """Background thread: periodically fetch QNH and update the altitude correction."""
    global qnh_correction_m
    while True:
        try:
            qnh = fetch_qnh(lat, lon)
            if qnh is not None:
                correction = (qnh - STANDARD_QNH) * HPFT * FT_TO_M
                with qnh_lock:
                    qnh_correction_m = correction
                print(f"[qnh] QNH={qnh:.1f} hPa, altitude correction={correction:+.1f}m")
        except Exception as e:
            print(f"[qnh] Could not fetch QNH: {e}")
        time.sleep(QNH_UPDATE_INTERVAL)


# ── Main ─────────────────────────────────────────────────────────────────────


def parse_args():
    parser = argparse.ArgumentParser(
        description="ADS-B to rotator controller — track aircraft with an antenna rotator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Example: python3 adsb_rotctl.py 192.168.1.50:4533",
    )
    parser.add_argument(
        "rotator",
        help="Rotator address as IP:PORT (e.g., 192.168.1.50:4533)",
    )
    parser.add_argument("--icao", help="Start tracking a specific aircraft by ICAO hex")
    parser.add_argument("--lat", type=float, default=None, help="Receiver latitude (default: from receiver.json)")
    parser.add_argument("--lon", type=float, default=None, help="Receiver longitude (default: from receiver.json)")
    parser.add_argument("--alt", type=float, default=None, help="Receiver altitude in meters ASL (default: auto-detect via Open-Meteo)")
    parser.add_argument("--sbs-port", type=int, default=30003, help="SBS/BaseStation TCP port on localhost (default: 30003)")
    parser.add_argument("--sbs-host", default="localhost", help="SBS/BaseStation host (default: localhost)")
    parser.add_argument("--url", default="http://localhost/tar1090", help="tar1090 base URL for receiver.json (default: http://localhost/tar1090)")
    parser.add_argument("--listen-port", type=int, default=7878, help="HTTP port for tar1090 selection events (default: 7878)")
    return parser.parse_args()


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


def main():
    global target_icao
    args = parse_args()

    rot_host, rot_port = parse_rotator_address(args.rotator)
    base_url = args.url.rstrip("/")

    # Resolve receiver location
    rx_lat, rx_lon = args.lat, args.lon
    if rx_lat is None or rx_lon is None:
        print("[info] Fetching receiver location from receiver.json...")
        json_dir = detect_json_dir()
        auto_lat, auto_lon = get_receiver_location(json_dir, base_url)
        if auto_lat is not None:
            rx_lat = rx_lat if rx_lat is not None else auto_lat
            rx_lon = rx_lon if rx_lon is not None else auto_lon
            print(f"[info] Receiver location: {rx_lat:.4f}, {rx_lon:.4f}")
        else:
            print("Error: could not determine receiver location.", file=sys.stderr)
            print("Provide --lat and --lon, or ensure receiver.json is accessible.", file=sys.stderr)
            sys.exit(1)

    rx_alt = args.alt
    if rx_alt is None:
        try:
            rx_alt = fetch_elevation(rx_lat, rx_lon)
            if rx_alt is not None:
                print(f"[info] Receiver elevation (Open-Meteo): {rx_alt:.0f}m ASL")
            else:
                rx_alt = 0.0
                print("[warning] Could not fetch elevation, defaulting to 0m ASL")
        except Exception as e:
            rx_alt = 0.0
            print(f"[warning] Could not fetch elevation ({e}), defaulting to 0m ASL")

    # Start QNH correction thread
    qnh_thread = threading.Thread(target=qnh_updater, args=(rx_lat, rx_lon), daemon=True)
    qnh_thread.start()

    # Set initial ICAO from CLI
    if args.icao:
        with target_lock:
            target_icao = args.icao.lower().strip()
        print(f"[info] Initial target: {args.icao}")

    # Start HTTP server for tar1090 selection events
    httpd = HTTPServer(("0.0.0.0", args.listen_port), SelectionHandler)
    http_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    http_thread.start()
    print(f"[info] Listening for tar1090 selections on port {args.listen_port}")

    # Connect to rotctld
    rotctl = RotctlConnection(rot_host, rot_port)
    try:
        rotctl.connect()
    except OSError as e:
        print(f"[rotctld] Initial connection failed: {e}")
        print("[rotctld] Will retry on first position update...")

    # Aircraft state table
    table = AircraftTable()

    def on_position_update(icao):
        pass  # Kalman tracker polls at 10 Hz; no event signaling needed

    # Start SBS reader thread
    sbs_thread = threading.Thread(
        target=sbs_reader,
        args=(args.sbs_host, args.sbs_port, table, on_position_update),
        daemon=True,
    )
    sbs_thread.start()
    print(f"[info] Reading real-time SBS data from {args.sbs_host}:{args.sbs_port}")

    # Periodic stale aircraft pruning
    def prune_loop():
        while True:
            time.sleep(10)
            table.prune_stale()

    prune_thread = threading.Thread(target=prune_loop, daemon=True)
    prune_thread.start()

    print("[info] Running — press Ctrl+C to stop")

    tracker = IMMTracker(rx_lat, rx_lon, rx_alt)
    last_status = None
    last_status_time = 0.0
    tick_interval = 0.1  # 10 Hz

    try:
        while True:
            tick_start = time.monotonic()
            now = tick_start

            with target_lock:
                icao = target_icao

            ac = None
            ac_icao = None
            if icao:
                ac = table.get_aircraft(icao)
                ac_icao = icao
                # Selected aircraft lost — fall back to closest
                if ac is None:
                    print(f"[track] {icao} lost — falling back to closest aircraft")
                    with target_lock:
                        target_icao = None
                    icao = None

            if ac is None:
                result = table.get_closest(rx_lat, rx_lon)
                if result:
                    ac_icao, ac = result

            if ac is None:
                if now - last_status_time >= 0.5:
                    status = "No aircraft found"
                    if status != last_status:
                        print(f"[track] {status}")
                        last_status = status
                    last_status_time = now
                elapsed = time.monotonic() - tick_start
                time.sleep(max(0, tick_interval - elapsed))
                continue

            tracker.switch_target(ac_icao)
            tracker.process_measurement(ac, now)
            az, el = tracker.get_predicted_azel(now)

            if az is not None:
                rotctl.send_position(az, el)

                flight = (ac.get("flight") or "").strip()
                dist_km = haversine(rx_lat, rx_lon, ac["lat"], ac["lon"]) / 1000
                status = (
                    f"{ac_icao}"
                    f"{' (' + flight + ')' if flight else ''}"
                    f" AZ={az:6.2f} EL={el:5.2f}"
                    f" dist={dist_km:.1f}km alt={ac['alt']:.0f}m"
                )
                if status != last_status:
                    print(f"[track] {status}")
                    last_status = status

            elapsed = time.monotonic() - tick_start
            time.sleep(max(0, tick_interval - elapsed))

    except KeyboardInterrupt:
        print("\n[info] Shutting down...")
    finally:
        rotctl.close()
        httpd.shutdown()


if __name__ == "__main__":
    main()
