import sys; sys.path.insert(0, 'tools')
import numpy as np, pandas as pd
from scipy.ndimage import median_filter
z = np.load(sys.argv[1]); R = pd.read_csv(sys.argv[2])
j, k = z['j'], z['k']
yw = R.yaw.values[j]
df_ref = np.cos(yw) * (R.x.values[k] - R.x.values[j]) + np.sin(yw) * (R.y.values[k] - R.y.values[j])
vo = z['fwd']; vos = median_filter(vo, 9)
m = df_ref > 0.05
print(f'forward per pair: ref median {np.median(df_ref):.4f} m, VO median {np.median(vo):.4f}, ratio median {np.median(vo[m] / df_ref[m]):.4f}')
e = vos - df_ref
print(f'VO(median 9) - ref per pair: mean {e.mean()*1000:+.2f} mm rms {np.sqrt(np.mean(e**2))*1000:.2f} mm; distance ref {df_ref.sum():.1f} m VO {vos.sum():.1f} m ({(vos.sum()/df_ref.sum()-1)*100:+.2f} %)')
for s0 in range(0, len(vo), 500):
    s = slice(s0, s0 + 500)
    print(f'   {s0:4d}: ref {df_ref[s].sum():7.2f} m  VO {vos[s].sum():7.2f} m  ({(vos[s].sum()/max(df_ref[s].sum(),1e-6)-1)*100:+.1f} %) resp median {np.median(z["resp"][s]):.3f}')
