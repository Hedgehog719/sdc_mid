"""Pick fusion noise settings by covariance consistency (NIS ~ chi2(2): mean 2), no leaderboard information."""
import itertools, subprocess, sys, re, json
import numpy as np
from concurrent.futures import ThreadPoolExecutor
T, base, grid, out = sys.argv[1], sys.argv[2], json.loads(sys.argv[3]), sys.argv[4]
def run(v):
    p = dict(zip(grid, v))
    args = base.format(**p)
    f = 'work/sweep/nis_' + T + '_' + '_'.join(f'{k}{p[k]}' for k in grid) + '.csv'
    r = subprocess.run(f'python3 tools/fuse.py /root/data/{T} {f} {args} --nis', shell=True, capture_output=True, text=True).stdout
    nis = {m.group(1): (float(m.group(2)), float(m.group(3))) for m in re.finditer(r'NIS (\w+)\s+n\s+\d+\s+mean\s+([\d.]+).*?share > 9.2:\s+([\d.]+)', r)}
    cost = sum(abs(np.log(nis[k][0] / 2.0)) for k in ('gnss', 'bev') if k in nis)
    return dict(p=p, nis=nis, cost=cost, file=f)
R = list(ThreadPoolExecutor(6).map(run, list(itertools.product(*grid.values()))))
R.sort(key=lambda r: r['cost'])
for r in R[:8]:
    print(f"cost {r['cost']:.3f} {r['p']} " + ' '.join(f"{k} {v[0]:.2f}/{v[1]:.1f}%" for k, v in r['nis'].items()))
json.dump(R, open(out, 'w'), indent=0)
