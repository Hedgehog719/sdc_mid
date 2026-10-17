"""Yaw as IMU yaw + a slowly varying offset b(t), estimated by a scalar Kalman filter + RTS smoother.

Measurements of the true yaw (heading of travel):
  * motion heading from a trajectory (displacement over a baseline), sigma = sqrt(2) * sigma_pos / (v * dt_base),
    so slow driving is down-weighted automatically (instead of a fixed Gaussian low-pass);
  * optional BEV map-to-image yaw fixes (lane-marking alignment), Huber-gated.
"""
import numpy as np


def yaw_offset_rts(t_out, imu_yaw_fn, meas_t, meas_yaw, meas_sigma, q_b_deg=0.05, huber=2.5, b0=0.0):
    """t_out, meas_t in ns. Returns smoothed offset b at t_out (rad)."""
    t_out = np.asarray(t_out, np.float64)
    meas_t = np.asarray(meas_t, np.float64)
    z = np.unwrap(meas_yaw) - imu_yaw_fn(meas_t)
    z = (z + np.pi) % (2 * np.pi) - np.pi
    times = np.concatenate([t_out, meas_t])
    kind = np.concatenate([np.zeros(len(t_out), int), np.ones(len(meas_t), int)])
    ref = np.concatenate([np.arange(len(t_out)), np.arange(len(meas_t))])
    order = np.lexsort((kind, times))
    times, kind, ref = times[order], kind[order], ref[order]
    n = len(times)
    q = np.radians(q_b_deg) ** 2
    xs, Ps, xp, Pp = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    x, P = b0, np.radians(5.0) ** 2
    tp = times[0]
    for k in range(n):
        dt = max((times[k] - tp) * 1e-9, 0.0)
        P = P + q * dt
        xp[k], Pp[k] = x, P
        if kind[k] == 1:
            r = z[ref[k]] - x
            r = (r + np.pi) % (2 * np.pi) - np.pi
            R = meas_sigma[ref[k]] ** 2
            m = abs(r) / np.sqrt(P + R)
            if m > huber:
                R *= (m / huber) ** 2
            K = P / (P + R)
            x, P = x + K * r, (1 - K) * P
        xs[k], Ps[k] = x, P
        tp = times[k]
    xsm = xs.copy()
    for k in range(n - 2, -1, -1):
        C = Ps[k] / Pp[k + 1]
        xsm[k] = xs[k] + C * (xsm[k + 1] - xp[k + 1])
    out = np.zeros(len(t_out))
    out[ref[kind == 0]] = xsm[kind == 0]
    return out


def motion_heading(t_ns, xy, base_s=1.0, sigma_pos=0.15, min_speed=1.0):
    """Heading of travel from a trajectory by central displacement over +-base_s/2; returns t, yaw, sigma."""
    t = (np.asarray(t_ns, np.float64) - t_ns[0]) * 1e-9
    tq0 = np.interp(t - base_s / 2, t, np.arange(len(t)))
    tq1 = np.interp(t + base_s / 2, t, np.arange(len(t)))
    i0 = np.clip(np.round(tq0).astype(int), 0, len(t) - 1)
    i1 = np.clip(np.round(tq1).astype(int), 0, len(t) - 1)
    d = xy[i1] - xy[i0]
    dtb = np.maximum(t[i1] - t[i0], 1e-3)
    v = np.linalg.norm(d, axis=1) / dtb
    yaw = np.arctan2(d[:, 1], d[:, 0])
    sig = np.sqrt(2) * sigma_pos / np.maximum(v * dtb, 1e-3)
    ok = v > min_speed
    return np.asarray(t_ns)[ok], yaw[ok], sig[ok]
