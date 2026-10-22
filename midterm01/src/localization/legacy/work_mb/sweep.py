"""Local sweep of fusion settings; objective = estimated position part vs the scored files (reference model)."""
import itertools, subprocess, sys, os, json
import numpy as np, pandas as pd
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, 'work/mb')
from est_lib import Est
T = 'T2'
E = Est(T)
bevs = sys.argv[1].split(',')
out_json = sys.argv[2]
grid = json.loads(os.environ['GRID']) if 'GRID' in os.environ else dict(k=[0.005, 0.01, 0.02, 0.04], fs=[1.0, 2.0], pf=[0.25, 0.4, 0.8], gs=[0.1, 0.2, 0.4], gt=[10, 30])
def common(bev, k, fs, pf, gs, gt, ls=0.5):
    return (f'--yaw_src imu_cf --yaw_cf_ref motion --yaw_cf_sigma 30 --rts --bev {bev} --bev_lat_sigma {ls} --bev_fwd_sigma {fs} --bev_huber 1.5 '
            f'--bev_peak_ref 0.2 --bev_min_speed 0 --bev_aniso_k {k} --bev_aniso_min 0.3 --bev_aniso_lat --pole work/pc_pm_t2.npz --pole_pct 60 '
            f'--pole_fwd_sigma {pf} --pole_lat_sigma 0.6 --road work/c_rd_t2.npz --road_pct 50 --road_lat_sigma 0.5 --yaw_rts traj --yaw_rts_q 0.05 '
            f'--yaw_rts_bev 0.7 --gnss_bias_tau {gt} --gnss_bias_sigma {gs} --gnss_white 0.03')
os.makedirs('work/sweep', exist_ok=True)
def p1(bev):
    tag = os.path.basename(bev)[:-4]
    f = f'work/sweep/{tag}_p1.csv'
    if not os.path.exists(f):
        subprocess.run(f'python3 tools/fuse.py /root/data/Track2 {f} {common(bev, 0.01, 2.0, 0.4, 0.2, 10)} --rts_sigma 0.3 --rts_qpos 0.4 --rts_qlat 0.1', shell=True, capture_output=True)
    return f
def run(args):
    bev, k, fs, pf, gs, gt = args[:6]; ls = args[6] if len(args) > 6 else 0.5
    tag = os.path.basename(bev)[:-4]
    f = f'work/sweep/{tag}_k{k}_fs{fs}_pf{pf}_gs{gs}_gt{gt}' + (f'_ls{ls}' if ls != 0.5 else '') + '.csv'
    if not os.path.exists(f):
        subprocess.run(f'python3 tools/fuse.py /root/data/Track2 {f} {common(*args)} --rts_sigma 0.3 --rts_qpos 0.1 --rts_qlat 0.05 --psi_csv {P1[bev]}', shell=True, capture_output=True)
    P, sh = E.pos(pd.read_csv(f))
    return dict(file=f, bev=tag, k=k, fs=fs, pf=pf, gs=gs, gt=gt, ls=ls, P=P, share=sh)
P1 = {b: p1(b) for b in bevs}
cfgs = [(b, *v) for b in bevs for v in itertools.product(*grid.values())]
with ThreadPoolExecutor(6) as ex:
    R = list(ex.map(run, cfgs))
R.sort(key=lambda r: r['P'])
json.dump(R, open(out_json, 'w'), indent=0)
for r in R[:12]: print(f"P {r['P']:.4f} (span {r['share']*100:.0f}%) {r['bev']} k {r['k']} fs {r['fs']} pf {r['pf']} gs {r['gs']} gt {r['gt']} ls {r['ls']}")
print('worst', R[-1]['P'], 'n', len(R))
