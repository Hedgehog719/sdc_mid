"""Kalman filter + RTS smoother for planar motion with known heading.

State s = [x, y, v]; heading psi(t) comes from the IMU (complementary-filtered), so the model is linear:
    x += v cos(psi) dt,  y += v sin(psi) dt,  v += a dt   (a ~ white noise)
plus a small white position noise for heading error / side slip. GNSS x/y are measurements at their own
timestamps; outputs are smoothed states at the requested query times. Unlike low-pass filtering this does
not cut corners, since the path shape follows the IMU heading.
"""
import numpy as np


def rts_smooth(t_query, t_meas, z_meas, psi_fn, sigma_meas=0.3, q_acc=1.0, q_pos=0.05, huber=3.0,
               meas_sigma=None, meas_cov=None, q_lat=None, t_zupt=None, sigma_zupt=0.02,
               gnss_bias=None, gnss_mask=None):
    """meas_cov: optional (N, 2, 2) per-measurement covariance (overrides sigma), e.g. lateral-only fixes.

    gnss_bias: (tau_s, sigma_b, sigma_w) models the GNSS error as coloured noise: a first-order Gauss-Markov bias
    state (correlation time tau_s, steady-state sigma_b, per axis) plus white noise sigma_w. Rows of t_meas flagged in
    gnss_mask observe position + bias; the other rows (map fixes) observe the position itself. A white-noise GNSS
    model treats 4 Hz samples as independent and over-trusts their average (the error of real GNSS drifts with a
    correlation time of seconds); with the bias states, map fixes calibrate the drift and GNSS keeps contributing
    its accurate relative motion."""
    # q_lat: cross-track position noise (non-holonomic constraint: no side slip); q_pos is then along-track.
    # t_zupt: times of zero-velocity pseudo-measurements (vehicle stationary, e.g. red lights).
    tq = np.asarray(t_query, np.float64)
    tm = np.asarray(t_meas, np.float64)
    tz = np.asarray([] if t_zupt is None else t_zupt, np.float64)
    times = np.concatenate([tq, tm, tz])
    kind = np.concatenate([np.zeros(len(tq), int), np.ones(len(tm), int), np.full(len(tz), 2)])
    ref = np.concatenate([np.arange(len(tq)), np.arange(len(tm)), np.arange(len(tz))])
    order = np.lexsort((kind, times))
    times, kind, ref = times[order], kind[order], ref[order]
    psi = psi_fn(times)
    n = len(times)
    nb = 2 if gnss_bias is not None else 0
    d = 3 + nb
    if nb:
        tau_b, sig_b, sig_w = gnss_bias
        gmask = np.zeros(len(tm), bool) if gnss_mask is None else np.asarray(gnss_mask, bool)
    xs = np.zeros((n, d))
    Ps = np.zeros((n, d, d))
    xp = np.zeros((n, d))
    Pp = np.zeros((n, d, d))
    Fs = np.zeros((n, d, d))
    # init from first two measurements
    first = np.argsort(tm)[:5]
    v0 = np.linalg.norm(z_meas[first[-1]] - z_meas[first[0]]) / max((tm[first[-1]] - tm[first[0]]) * 1e-9, 1e-3)
    x = np.zeros(d)
    x[:3] = [z_meas[first[0], 0], z_meas[first[0], 1], v0]
    P = np.diag([1.0, 1.0, 4.0] + ([sig_b ** 2] * nb if nb else []))
    H = np.zeros((2, d))
    H[0, 0] = H[1, 1] = 1.0
    Hg = H.copy()
    if nb:
        Hg[0, 3] = Hg[1, 4] = 1.0
    Hv = np.zeros((1, d))
    Hv[0, 2] = 1.0
    I = np.eye(d)
    t_prev = times[0]
    for k in range(n):
        dt = max((times[k] - t_prev) * 1e-9, 0.0)
        c, s = np.cos(psi[k]), np.sin(psi[k])
        F = np.eye(d)
        F[0, 2], F[1, 2] = c * dt, s * dt
        G = np.zeros(d)
        G[:3] = [c * dt * dt / 2, s * dt * dt / 2, dt]
        if q_lat is None:
            Qp = np.eye(2) * q_pos ** 2
        else:
            Rh = np.array([[c, -s], [s, c]])
            Qp = Rh @ np.diag([q_pos ** 2, q_lat ** 2]) @ Rh.T
        Q = q_acc ** 2 * np.outer(G, G)
        Q[:2, :2] += Qp * dt
        Q[2, 2] += 1e-6 * dt
        if nb:  # GNSS bias: first-order Gauss-Markov
            phi = np.exp(-dt / tau_b)
            F[3, 3] = F[4, 4] = phi
            Q[3, 3] += sig_b ** 2 * (1 - phi * phi) + 1e-12
            Q[4, 4] += sig_b ** 2 * (1 - phi * phi) + 1e-12
        x = F @ x
        P = F @ P @ F.T + Q
        Fs[k], xp[k], Pp[k] = F, x, P
        if kind[k] == 2:  # zero-velocity update
            S = Hv @ P @ Hv.T + sigma_zupt ** 2
            K = P @ Hv.T / S
            x = x + (K * (0.0 - x[2])).ravel()
            P = (I - K @ Hv) @ P
        if kind[k] == 1:
            z = z_meas[ref[k]]
            is_g = nb and gmask[ref[k]]
            Hm = Hg if is_g else H
            if is_g:
                R = np.eye(2) * sig_w ** 2
            elif meas_cov is not None:
                R = meas_cov[ref[k]].copy()
            else:
                sm = sigma_meas if meas_sigma is None else meas_sigma[ref[k]]
                R = np.eye(2) * sm ** 2
            S = Hm @ P @ Hm.T + R
            r = z - Hm @ x
            m = np.sqrt(r @ np.linalg.solve(S, r))
            if m > huber:  # robust: inflate noise of outlying fixes
                R = R * (m / huber) ** 2
                S = Hm @ P @ Hm.T + R
            K = P @ Hm.T @ np.linalg.inv(S)
            x = x + K @ r
            P = (I - K @ Hm) @ P
        xs[k], Ps[k] = x, P
        t_prev = times[k]
    # RTS backward pass
    xsm = xs.copy()
    for k in range(n - 2, -1, -1):
        C = Ps[k] @ Fs[k + 1].T @ np.linalg.inv(Pp[k + 1])
        xsm[k] = xs[k] + C @ (xsm[k + 1] - xp[k + 1])
    out = np.zeros((len(tq), 3))
    q = kind == 0
    out[ref[q]] = xsm[q][:, :3]
    return out
