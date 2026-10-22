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
               gnss_bias=None, gnss_mask=None, map_bias=None, map_mask=None, return_nis=False, huber_g=None,
               acc_fn=None, gnss_lat_extra=0.0, gnss_fwd_extra=0.0):
    """meas_cov: optional (N, 2, 2) per-measurement covariance (overrides sigma), e.g. lateral-only fixes.

    gnss_bias: (tau_s, sigma_b, sigma_w) models the GNSS error as coloured noise: a first-order Gauss-Markov bias
    state (correlation time tau_s, steady-state sigma_b, per axis) plus white noise sigma_w. Rows of t_meas flagged in
    gnss_mask observe position + bias; the other rows (map fixes) observe the position itself. A white-noise GNSS
    model treats 4 Hz samples as independent and over-trusts their average (the error of real GNSS drifts with a
    correlation time of seconds); with the bias states, map fixes calibrate the drift and GNSS keeps contributing
    its accurate relative motion.

    map_bias: (L_m, sigma_m) models the common error of the map-derived fixes (lane / kerb / pole matches are all
    registered to the same point-cloud map) as a first-order Gauss-Markov process in travelled distance (correlation
    length L_m, steady-state sigma_m per axis; constant while stationary). Rows flagged in map_mask observe position +
    map bias. Without it the smoother treats map fixes as unbiased and copies the map's local distortions; block
    cross-validation cannot see that error (it is common to all map fixes), the leaderboard can."""
    # acc_fn(t) -> (N, 2) world-frame horizontal acceleration (IMU, gravity removed): used as a known input to the
    # along-track speed (projected on the heading); q_acc then only covers the IMU's own error.
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
    nm = 2 if map_bias is not None else 0
    d = 3 + nb + nm
    jm = 3 + nb  # first map-bias state
    if nb:
        tau_b, sig_b, sig_w = gnss_bias
        gmask = np.zeros(len(tm), bool) if gnss_mask is None else np.asarray(gnss_mask, bool)
    if nm:   # (L, sigma) isotropic, or (L, sigma_along, sigma_cross): along / across the heading
        len_m, sig_m = map_bias[0], map_bias[1]
        sig_mc = map_bias[2] if len(map_bias) > 2 else sig_m
        path_frame = len(map_bias) > 2   # anisotropic: the state is (along-track, cross-track), rotated by the heading
                                         # in the measurement (a world-frame state would turn forward error into
                                         # lateral error after every bend for long correlation lengths)
        mmask = np.zeros(len(tm), bool) if map_mask is None else np.asarray(map_mask, bool)
    if not nm:
        path_frame = False
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
    P = np.diag([1.0, 1.0, 4.0] + ([sig_b ** 2] * nb if nb else []) + ([sig_m ** 2] * nm if nm else []))
    if nm:
        P[jm:jm + 2, jm:jm + 2] = np.diag([sig_m ** 2, sig_mc ** 2])
    H = np.zeros((2, d))
    H[0, 0] = H[1, 1] = 1.0
    Hg = H.copy()
    if nb:
        Hg[0, 3] = Hg[1, 4] = 1.0
    Hmap = H.copy()
    if nm:
        Hmap[0, jm] = Hmap[1, jm + 1] = 1.0
    Hv = np.zeros((1, d))
    Hv[0, 2] = 1.0
    I = np.eye(d)
    nis = np.full(len(tm), np.nan)  # normalised innovation squared per measurement (before Huber)
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
        if nm:  # map bias: Gauss-Markov in travelled distance (speed from the current estimate)
            phm = np.exp(-abs(x[2]) * dt / len_m)
            F[jm, jm] = F[jm + 1, jm + 1] = phm
            Q[jm:jm + 2, jm:jm + 2] += np.diag([sig_m ** 2, sig_mc ** 2]) * (1 - phm * phm) + np.eye(2) * 1e-12
        x = F @ x
        if acc_fn is not None and dt > 0:
            aw = acc_fn(np.array([(times[k] + t_prev) / 2]))[0]
            x = x + G * (c * aw[0] + s * aw[1])
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
            is_m = nm and mmask[ref[k]]
            Hm = Hg if is_g else (Hmap if is_m else H)
            if is_m and path_frame:
                Hm = Hmap.copy()
                Hm[:, jm:jm + 2] = np.array([[c, -s], [s, c]])
            if is_g:
                R = np.eye(2) * sig_w ** 2
                if gnss_fwd_extra > 0:   # GNSS forward worse than its lateral: inflate the along-track part only
                    fv = np.array([c, s])
                    R = R + gnss_fwd_extra ** 2 * np.outer(fv, fv)
                if gnss_lat_extra > 0:   # GNSS lateral much worse than forward (biased across the road): inflate it only
                    lv = np.array([-s, c])
                    R = R + gnss_lat_extra ** 2 * np.outer(lv, lv)
            elif meas_cov is not None:
                R = meas_cov[ref[k]].copy()
            else:
                sm = sigma_meas if meas_sigma is None else meas_sigma[ref[k]]
                R = np.eye(2) * sm ** 2
            S = Hm @ P @ Hm.T + R
            r = z - Hm @ x
            m = np.sqrt(r @ np.linalg.solve(S, r))
            nis[ref[k]] = m * m
            hb = huber_g if (is_g and huber_g is not None) else huber
            if m > hb:  # robust: inflate noise of outlying fixes (GNSS may use its own threshold)
                R = R * (m / hb) ** 2
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
    if return_nis:
        return out, nis
    return out
