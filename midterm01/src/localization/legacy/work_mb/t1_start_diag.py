"""T1 start, local consistency only: fused trajectory vs the lane fixes and vs GNSS-only fusion, per segment."""
import sys, numpy as np, pandas as pd
fx = dict(np.load(sys.argv[1])); i = fx['idx'].astype(int)
G = pd.read_csv('work/_t1_gnssonly.csv')
for tr in sys.argv[2:]:
    T = pd.read_csv(tr); yw = T.yaw.values[i]
    dx, dy = fx['mx'] - T.x.values[i], fx['my'] - T.y.values[i]
    fw, lt = np.cos(yw) * dx + np.sin(yw) * dy, -np.sin(yw) * dx + np.cos(yw) * dy
    gx, gy = T.x.values - G.x.values, T.y.values - G.y.values
    gyw = T.yaw.values
    gf = np.cos(gyw) * gx + np.sin(gyw) * gy
    out = []
    for a, b in ((0, 110), (110, 220), (220, 330), (330, 600)):
        m = (i >= a) & (i < b)
        out.append(f'{a}-{b}: fix-fused fwd {np.median(fw[m]):+.2f} lat {np.median(lt[m]):+.2f}, fused-GNSSonly fwd {gf[a:b].mean():+.2f}')
    print(f'{tr.split("/")[-1]:24s} ' + ' | '.join(out))
