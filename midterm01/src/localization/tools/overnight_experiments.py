#!/usr/bin/env python3
"""Run a finite set of causal T3 model hypotheses with guarded submissions and per-result Git records."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

from submit_guarded import check_deadline, check_quota, validate_csv


PROFILES = {
    'conditioned_init': '--init_pos_per_hyp',
    'soft_heading': '--yaw_soft_peak 0.2',
    'conditioned_soft': '--init_pos_per_hyp --yaw_soft_peak 0.2',
    'stop_soft': '--still_meas 3 --yaw_soft_peak 0.2',
    'gyro_heading': '--gyro_heading --gyro_bias -0.266065785',
}
OFFSETS = ('0', '-1.5', '1.5', '-3', '3')
COMPETITION = 'sdc-2026-localization-track-03'


def read_rows(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def snapshot_runner(package, directory):
    destination = Path(directory) / 'runner_snapshot.sh'
    shutil.copyfile(Path(package) / 'tools' / 'run_online.sh', destination)
    return destination


def core_digest(package):
    digest = hashlib.sha256()
    for name in ('localize_online.py', 'bev.py', 'bev_match.py', 'lane_single.py', 'marking_map.py', 'camera_model.py'):
        digest.update((Path(package) / 'tools' / name).read_bytes())
    return digest.hexdigest()


def validate_prefix(directory, prefix_frames=600):
    directory = Path(directory)
    scores = []
    for offset in OFFSETS:
        rows = read_rows(directory / f'hyp_{offset}_dbg.csv')
        if len(rows) < prefix_frames:
            raise ValueError('incomplete heading hypothesis')
        scores.append(float(rows[prefix_frames - 1]['ll']) - 0.5 * (float(offset) / 2) ** 2)
    winner = max(range(len(OFFSETS)), key=lambda index: (scores[index], -abs(float(OFFSETS[index]))))
    short = read_rows(directory / f'hyp_{OFFSETS[winner]}.csv')
    full = read_rows(directory / 'full.csv')
    if min(len(short), len(full)) < prefix_frames:
        raise ValueError('incomplete causal prefix')
    for early, complete in zip(short[:prefix_frames], full[:prefix_frames]):
        if any(not math.isclose(float(early[key]), float(complete[key]), rel_tol=0, abs_tol=1e-10)
               for key in ('idx', 'x', 'y', 'yaw')):
            raise ValueError('full-run prefix differs from truncated hypothesis')
    return OFFSETS[winner]


def history():
    result = subprocess.run(['kaggle', 'competitions', 'submissions', '-c', COMPETITION, '--csv'],
                            check=True, capture_output=True, text=True)
    lines = result.stdout.splitlines()
    start = next(index for index, line in enumerate(lines) if line.startswith('ref,fileName,date,'))
    return result.stdout, list(csv.DictReader(lines[start:]))


def archive(package, repository, report, candidate):
    destination = package / 'submissions' / 'online' / candidate.name
    if candidate != destination:
        shutil.copyfile(candidate, destination)
    metadata = destination.with_suffix('.json')
    metadata.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    readme = package / 'README.md'
    text = readme.read_text()
    begin, end = '<!-- overnight-results:begin -->', '<!-- overnight-results:end -->'
    if begin not in text:
        text += ('\n### 6.6 Autonomous Batch Results\n\n'
                 'Fixed model hypotheses; live Docker matching, causal-prefix and CSV checks before guarded upload. '
                 'Scores below are Kaggle public scores, not local residuals.\n\n' + begin + '\n' + end + '\n')
    content_start = text.index(begin) + len(begin)
    content_end = text.index(end, content_start)
    rows = [line for line in text[content_start:content_end].splitlines()
            if line.startswith('| ') and not line.startswith('| Profile |') and not line.startswith('| ---')
            and not line.startswith(f"| {report['profile']} |")]
    rows.append(f"| {report['profile']} | `{report['extra']}` | {report['score']} | {report['ref']} | {report['status']} | `{destination.name}` |")
    table = '\n| Profile | Extra Options | Public | Ref | Status | File |\n| --- | --- | --- | --- | --- | --- |\n'
    text = text[:content_start] + table + '\n'.join(rows) + '\n' + text[content_end:]
    readme.write_text(text)
    paths = [str(path.relative_to(repository)) for path in (destination, metadata, readme)]
    subprocess.run(['git', 'add', '--'] + paths, cwd=repository, check=True)
    changed = subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=repository).returncode
    if changed:
        subprocess.run(['git', 'commit', '-m', f"Record T3 {report['profile']}: {report['status']} public {report['score']}"],
                       cwd=repository, check=True)
        subprocess.run(['git', 'push', 'origin', 'HEAD'], cwd=repository, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deadline', type=datetime.fromisoformat, required=True)
    parser.add_argument('--profiles', nargs='+', choices=PROFILES, default=list(PROFILES))
    parser.add_argument('--container', default='sdc_midterm01')
    parser.add_argument('--reserve', type=int, default=2)
    parser.add_argument('--dry-run', action='store_true')
    arguments = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    repository = package.parents[2]
    check_deadline(datetime.now(timezone.utc), arguments.deadline)
    if arguments.dry_run:
        print(json.dumps({profile: PROFILES[profile] for profile in arguments.profiles}, indent=2))
        return
    dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=repository, text=True)
    if dirty.strip():
        raise RuntimeError('commit current changes before starting the batch')
    for profile in arguments.profiles:
        try:
            check_deadline(datetime.now(timezone.utc), arguments.deadline, 1800)
            listing, _ = history()
            remaining = check_quota(listing, datetime.now(timezone.utc), reserve=arguments.reserve)
        except ValueError as error:
            print(f'STOP: {error}', flush=True)
            break
        revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repository, text=True).strip()
        relative = f'work/runs/T3_batch_{profile}'
        directory = package / relative
        directory.mkdir(parents=True, exist_ok=True)
        runner = snapshot_runner(package, directory)
        source_digest = core_digest(package)
        candidate = package / 'submissions' / 'online' / f't3_batch_{profile}.csv'
        if candidate.exists():
            raise RuntimeError(f'refuse duplicate candidate filename: {candidate}')
        print(f'START {profile}; quota {remaining}; revision {revision}', flush=True)
        command = ['docker', 'exec', '-w', '/root/midterm/src/localization', '-e', 'SDC_CAM_SYNC=1',
                   '-e', 'SDC_CAM_SYNC_YAW=1', '-e', 'SDC_HYP_WORKERS=1', '-e', f'SDC_RUN_DIR={relative}',
                   '-e', 'SDC_PACKAGE_DIR=/root/midterm/src/localization',
                   '-e', f'SDC_LOC_EXTRA={PROFILES[profile]}', arguments.container,
                   'bash', f'/root/midterm/src/localization/{runner.relative_to(package)}', '/root/data/Track3', 'T3']
        with (directory / 'run.log').open('w') as stream:
            subprocess.run(command, cwd=repository, stdout=stream, stderr=subprocess.STDOUT,
                           stdin=subprocess.DEVNULL, check=True)
        if core_digest(package) != source_digest:
            raise RuntimeError('core source changed during the run; refuse mixed-version submission')
        output = directory / 'T3_online.csv'
        validate_csv(output)
        winner = validate_prefix(directory)
        shutil.copyfile(output, candidate)
        report = {'profile': profile, 'extra': PROFILES[profile], 'revision': revision,
                  'core_sha256': source_digest,
                  'deadline': arguments.deadline.isoformat(), 'winner_offset': winner,
                  'sha256': hashlib.sha256(candidate.read_bytes()).hexdigest(),
                  'status': 'validated', 'score': '', 'ref': ''}
        archive(package, repository, report, candidate)
        subprocess.run([sys.executable, str(package / 'tools' / 'submit_guarded.py'), str(candidate),
                        '--deadline', arguments.deadline.isoformat(), '--reserve', str(arguments.reserve),
                        '-m', f'{revision[:7]} {profile}: live causal bank; prefix+CSV verified'], check=True)
        report.update(status='submitted', score='pending')
        archive(package, repository, report, candidate)
        _, entries = history()
        matches = [entry for entry in entries if entry['fileName'] == candidate.name]
        if matches:
            report.update(status=matches[0]['status'], score=matches[0]['publicScore'] or 'pending', ref=matches[0]['ref'])
            archive(package, repository, report, candidate)
        print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()