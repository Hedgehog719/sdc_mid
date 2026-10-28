# tempered posterior mean of the T3 heading bank (validation): weights from LL / TAU + the offset prior
import sys
import numpy as np
import pandas as pd
sys.path.insert(0, 'tools')
import hyp_select as HS  # noqa: E402
W, TAU, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
OFFS = [0, -1.5, 1.5, -3, 3]
NH = 600
S = np.array([pd.read_csv(f'{W}/hyp_{o:g}_dbg.csv')['ll'].values[:NH].astype(float) for o in OFFS]) / TAU
P = np.array([pd.read_csv(f'{W}/hyp_{o:g}.csv')[['x', 'y', 'yaw']].values[:NH] for o in OFFS])
mean = HS.posterior_mean(S, OFFS, P)
full = pd.read_csv(f'{W}/full.csv')
full.loc[:NH - 1, ['x', 'y', 'yaw']] = mean
full.to_csv(out, index=False)
