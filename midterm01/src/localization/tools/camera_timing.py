#!/usr/bin/env python3
"""Extract image header capture times and match each camera's filename convention exactly."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


CAMERAS = ('f', 'fl', 'fr')


def read_capture_times(path):
    timestamps = {camera: {} for camera in CAMERAS}
    with Path(path).open(newline='') as stream:
        for row in csv.DictReader(stream):
            camera, received, captured = row['camera'], int(row['receive_ns']), int(row['capture_ns'])
            if camera not in timestamps or received in timestamps[camera]:
                raise ValueError('unknown camera or duplicate receive timestamp')
            if captured <= 0 or captured > received:
                raise ValueError('capture time must be positive and no later than receive time')
            timestamps[camera][received] = captured
    return timestamps


def capture_arrays(track_dir, cameras=CAMERAS, metadata=None):
    root = Path(track_dir)
    track = root.name
    timestamps = read_capture_times(metadata or root / 'sensors' / 'camera_timing.csv')
    output = {}
    for camera in cameras:
        names = sorted((root / f'{track}_images' / f'{track}_{camera}_images').iterdir())
        filename_times = [int(name.name.split('_ts_')[1].split('.')[0]) for name in names]
        if not filename_times:
            raise ValueError(f'no images for {camera}')
        mapping = timestamps[camera]
        receive_match = all(timestamp in mapping for timestamp in filename_times)
        capture_values = set(mapping.values())
        capture_match = all(timestamp in capture_values for timestamp in filename_times)
        if receive_match and capture_match and any(mapping[timestamp] != timestamp for timestamp in filename_times):
            raise ValueError(f'ambiguous filename timestamp convention for {camera}')
        if receive_match:
            captured = np.array([mapping[timestamp] for timestamp in filename_times], dtype=np.int64)
        elif capture_match:
            captured = np.array(filename_times, dtype=np.int64)
        else:
            raise ValueError(f'missing exact timestamp convention for {camera}; no interpolation allowed')
        if np.any(np.diff(captured) <= 0):
            raise ValueError('capture timestamps must increase in frame order')
        output[camera] = captured
    return output


def capture_offsets(track_dir, cameras=CAMERAS):
    requested = tuple(dict.fromkeys(('f',) + tuple(cameras)))
    captured = capture_arrays(track_dir, cameras=requested)
    if any(len(values) != len(captured['f']) for values in captured.values()):
        raise ValueError('all camera frames must align by index')
    return {camera: (captured[camera] - captured['f']) * 1e-9 for camera in cameras}


def retime_vo_pairs(values, captured):
    required = ('j', 'k', 'fwd', 'lat', 'resp')
    if any(name not in values for name in required):
        raise ValueError('VO cache must contain frame pairs, displacement and quality')
    previous, current = np.asarray(values['j']), np.asarray(values['k'])
    captured = np.asarray(captured, dtype=np.int64)
    if previous.ndim != 1 or current.shape != previous.shape or not len(previous):
        raise ValueError('expected nonempty one-dimensional VO pairs')
    if not np.issubdtype(previous.dtype, np.integer) or not np.issubdtype(current.dtype, np.integer):
        raise ValueError('frame indices must be integers')
    if np.any(previous < 0) or np.any(current >= len(captured)) or np.any(current - previous != 1):
        raise ValueError('only valid adjacent causal frame pairs may be retimed')
    if np.any(np.diff(captured) <= 0):
        raise ValueError('capture timestamps must increase')
    for name in ('fwd', 'lat', 'resp'):
        if np.asarray(values[name]).shape != previous.shape or not np.isfinite(values[name]).all():
            raise ValueError('displacements and qualities must be finite and pair-aligned')
    interval = (captured[current] - captured[previous]) * 1e-9
    if not np.isfinite(interval).all() or np.any(interval <= 0):
        raise ValueError('capture pair intervals must be positive')
    result = {name: np.asarray(value).copy() for name, value in values.items()}
    result.update(dt=interval, speed=result['fwd'] / interval,
                  timing_reference=np.array('front_camera_header_capture'))
    return result


def extract(track_dir, output):
    from rosbags.highlevel import AnyReader
    root = Path(track_dir)
    with AnyReader([root]) as reader:
        selected = {}
        for connection in reader.connections:
            for camera in CAMERAS:
                if f'gige_100_{camera}_hdr' in connection.topic and connection.topic.endswith('/h265'):
                    selected[connection.id] = camera
        if set(selected.values()) != set(CAMERAS):
            raise ValueError('all three image topics are required')
        with Path(output).open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['camera', 'receive_ns', 'capture_ns'])
            connections = [connection for connection in reader.connections if connection.id in selected]
            for connection, received, raw in reader.messages(connections=connections):
                message = reader.deserialize(raw, connection.msgtype)
                captured = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                writer.writerow([selected[connection.id], received, captured])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('track_dir', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--vo', type=Path, help='optional existing displacement cache to retime without rematching')
    parser.add_argument('--vo-out', type=Path)
    arguments = parser.parse_args()
    if bool(arguments.vo) != bool(arguments.vo_out):
        parser.error('--vo and --vo-out must be supplied together')
    output = arguments.out or arguments.track_dir / 'sensors' / 'camera_timing.csv'
    if not output.exists():
        extract(arguments.track_dir, output)
    arrays = capture_arrays(arguments.track_dir, metadata=output)
    for camera, captured in arrays.items():
        offset = (captured - arrays['f']) * 1e-6
        print(camera, 'matched frames', len(captured), 'capture offset ms p0/p50/p95/p100',
              np.percentile(offset, [0, 50, 95, 100]), flush=True)
    if arguments.vo:
        with np.load(arguments.vo) as archive:
            original = {name: archive[name].copy() for name in archive.files}
        retimed = retime_vo_pairs(original, arrays['f'])
        np.savez(arguments.vo_out, **retimed)
        report = {'source': str(arguments.vo), 'source_sha256': hashlib.sha256(arguments.vo.read_bytes()).hexdigest(),
                  'output': str(arguments.vo_out), 'output_sha256': hashlib.sha256(arguments.vo_out.read_bytes()).hexdigest(),
                  'pairs': len(retimed['j']), 'changed_only': ['dt', 'speed', 'timing_reference'],
                  'all_displacements_and_qualities_identical': True, 'causal_pair_indices': True}
        arguments.vo_out.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
        print('retimed adjacent VO pairs', report['pairs'], 'without changing displacement or quality', flush=True)


if __name__ == '__main__':
    main()