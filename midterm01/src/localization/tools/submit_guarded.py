#!/usr/bin/env python3
"""Validate an online submission and refuse new uploads near an explicit timezone-aware deadline."""
import argparse
import csv
from datetime import datetime, timedelta, timezone
import math
from pathlib import Path
import subprocess


def check_deadline(now, deadline, buffer_seconds=120):
    if now.utcoffset() is None or deadline.utcoffset() is None:
        raise ValueError('deadline and current time must include a timezone')
    if buffer_seconds < 0:
        raise ValueError('buffer_seconds must not be negative')
    if now >= deadline - timedelta(seconds=buffer_seconds):
        raise ValueError('submission window closed; no upload started')


def validate_csv(path, expected_rows=3142):
    with Path(path).open(newline='') as stream:
        reader = csv.reader(stream)
        if next(reader, None) != ['idx', 'x', 'y', 'yaw']:
            raise ValueError('expected CSV header idx,x,y,yaw')
        count = 0
        for expected_index, row in enumerate(reader):
            if len(row) != 4 or int(row[0]) != expected_index:
                raise ValueError(f'invalid row index at row {expected_index}')
            values = list(map(float, row[1:]))
            if not all(map(math.isfinite, values)) or not -math.pi <= values[2] <= math.pi:
                raise ValueError(f'invalid pose at row {expected_index}')
            count += 1
    if count != expected_rows:
        raise ValueError(f'expected {expected_rows} poses, got {count}')


def check_quota(submissions_csv, now, maximum=15, reserve=0):
    if maximum < 1 or not 0 <= reserve < maximum:
        raise ValueError('invalid daily submission limit or reserve')
    lines = submissions_csv.splitlines()
    start = next((index for index, line in enumerate(lines) if line.startswith('ref,fileName,date,')), None)
    if start is None:
        raise ValueError('cannot verify submission quota: CSV header missing')
    used = 0
    for row in csv.DictReader(lines[start:]):
        submitted = datetime.fromisoformat(row['date'].replace('Z', '+00:00'))
        if submitted.utcoffset() is None:
            submitted = submitted.replace(tzinfo=timezone.utc)
        used += submitted.astimezone(timezone.utc).date() == now.astimezone(timezone.utc).date()
    remaining = maximum - used
    if remaining <= reserve:
        raise ValueError(f'submission quota exhausted or reserved: remaining {remaining}, reserve {reserve}')
    return remaining


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_file', type=Path)
    parser.add_argument('-c', '--competition', default='sdc-2026-localization-track-03')
    parser.add_argument('-m', '--message', required=True)
    parser.add_argument('--deadline', type=datetime.fromisoformat, required=True)
    parser.add_argument('--buffer-seconds', type=int, default=120)
    parser.add_argument('--rows', type=int, default=3142)
    parser.add_argument('--max-per-day', type=int, default=15)
    parser.add_argument('--reserve', type=int, default=0, help='leave this many daily submissions unused')
    arguments = parser.parse_args()
    try:
        check_deadline(datetime.now(timezone.utc), arguments.deadline, arguments.buffer_seconds)
        validate_csv(arguments.csv_file, arguments.rows)
        history = subprocess.run(['kaggle', 'competitions', 'submissions', '-c', arguments.competition, '--csv'],
                     check=True, capture_output=True, text=True)
        remaining = check_quota(history.stdout, datetime.now(timezone.utc), arguments.max_per_day, arguments.reserve)
        check_deadline(datetime.now(timezone.utc), arguments.deadline, arguments.buffer_seconds)
    except (ValueError, OSError) as error:
        parser.exit(2, f'{error}\n')
    print(f'Validated {arguments.csv_file}; remaining today {remaining}; upload deadline {arguments.deadline.isoformat()}', flush=True)
    subprocess.run(['kaggle', 'competitions', 'submit', '-c', arguments.competition,
                    '-f', str(arguments.csv_file), '-m', arguments.message], check=True)


if __name__ == '__main__':
    main()