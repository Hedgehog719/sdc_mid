#!/usr/bin/env python3
"""Local causal GNSS holdout validation for T1/T2; never uploads competition submissions."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess

import numpy as np
import pandas as pd

from overnight_experiments import core_digest, snapshot_runner
from submit_guarded import check_deadline


VARIANTS = ['baseline', 'translation', 'se2', 'gyro_heading', 'gnss_bias', 'gnss_bias_heading']


def variant_options(track, variant):
    sync = variant not in ('baseline', 'gnss_bias')
    yaw = variant in ('se2', 'gyro_heading', 'gnss_bias_heading')
    extra = ''
    if variant in ('gyro_heading', 'gnss_bias_heading'):
        bias = '-0.263836007' if track == 'T1' else '-0.293533891'
        extra += f' --gyro_heading --gyro_bias {bias}'
    if variant in ('gnss_bias', 'gnss_bias_heading'):
        extra += ' --gnss_bias_sd 0.25 --gnss_bias_tau 3'
    return sync, yaw, extra


def evaluate_holdout(frame_times, debug, samples):
    frame_times = np.asarray(frame_times, dtype=np.int64)
    if len(frame_times) != len(debug) or np.any(np.diff(frame_times) <= 0):
        raise ValueError('frame times and debug poses must match in strictly increasing time')
    indices = np.searchsorted(frame_times, samples.t.to_numpy(), side='right') - 1
    valid = (indices >= 0) & (samples.t.to_numpy() <= frame_times[-1])
    if not valid.any():
        raise ValueError('no held-out GNSS samples within output time range')
    selected = debug.iloc[indices[valid]]
    elapsed = (samples.t.to_numpy()[valid] - frame_times[indices[valid]]) * 1e-9
    heading = np.radians(selected.yaw.to_numpy())
    predicted = selected[['x', 'y']].to_numpy() + np.column_stack((np.cos(heading), np.sin(heading))) * (selected.v.to_numpy() * elapsed)[:, None]
    error = predicted - samples[['x', 'y']].to_numpy()[valid]
    if not np.isfinite(error).all():
        raise ValueError('nonfinite holdout residuals')
    forward = error[:, 0] * np.cos(heading) + error[:, 1] * np.sin(heading)
    lateral = -error[:, 0] * np.sin(heading) + error[:, 1] * np.cos(heading)
    return {'heldout_samples': int(valid.sum()),
            'position_rmse': float(np.sqrt((error ** 2).sum(axis=1).mean())),
            'forward_rmse': float(np.sqrt((forward ** 2).mean())),
            'lateral_rmse': float(np.sqrt((lateral ** 2).mean())),
            'mean_forward': float(forward.mean()), 'mean_lateral': float(lateral.mean())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tracks', nargs='+', choices=['T1', 'T2'], default=['T1', 'T2'])
    parser.add_argument('--variants', nargs='+', choices=VARIANTS,
                        default=['baseline', 'translation', 'se2', 'gyro_heading'])
    parser.add_argument('--frames', type=int, default=600)
    parser.add_argument('--phase', type=int, choices=[0, 1], default=0)
    parser.add_argument('--deadline', type=datetime.fromisoformat, required=True)
    arguments = parser.parse_args()
    from common import ply_frames
    package = Path(__file__).resolve().parents[1]
    os.chdir(package)
    summary = Path(f'work/runs/cross_track_validation_p{arguments.phase}.json')
    results = json.loads(summary.read_text()) if summary.exists() else []
    for track in arguments.tracks:
        folder = 'Track' + track[1:]
        for variant in arguments.variants:
            check_deadline(datetime.now(timezone.utc), arguments.deadline, 600)
            relative = Path(f'work/runs/{track}_validation_{variant}_p{arguments.phase}')
            relative.mkdir(parents=True, exist_ok=True)
            snapshot = snapshot_runner(package, package / relative)
            digest = core_digest(package)
            extra = f'--gnss_stride 2 --gnss_phase {arguments.phase} --frames {arguments.frames}'
            sync, yaw, model_options = variant_options(track, variant)
            extra += model_options
            environment = os.environ.copy()
            environment.update(SDC_RUN_DIR=str(relative), SDC_PACKAGE_DIR=str(package), SDC_LOC_EXTRA=extra,
                               SDC_CAM_SYNC='1' if sync else '0', SDC_CAM_SYNC_YAW='1' if yaw else '0')
            print(f'START {track} {variant} phase {arguments.phase}', flush=True)
            with (relative / 'run.log').open('w') as log:
                subprocess.run(['bash', str(snapshot), '/root/data/' + folder, track], env=environment,
                               stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, check=True)
            if core_digest(package) != digest:
                raise RuntimeError('core changed during holdout run')
            debug = pd.read_csv(relative / 'debug.csv')
            samples = pd.read_csv(f'/root/data/{folder}/sensors/gnss.csv').iloc[1 - arguments.phase::2]
            times = np.asarray(ply_frames(f'/root/data/{folder}/raw_ply')[0], dtype=np.int64)[:len(debug)]
            report = dict(track=track, variant=variant, phase=arguments.phase, frames=len(debug),
                          core_sha256=digest, **evaluate_holdout(times, debug, samples))
            (relative / 'holdout.json').write_text(json.dumps(report, indent=2) + '\n')
            results = [entry for entry in results if (entry['track'], entry['variant']) != (track, variant)]
            results.append(report)
            summary.write_text(json.dumps(results, indent=2) + '\n')
            print(json.dumps(report), flush=True)
    print('DONE local held-out GNSS validation; no Kaggle uploads; GNSS is not hidden ground truth', flush=True)


if __name__ == '__main__':
    main()