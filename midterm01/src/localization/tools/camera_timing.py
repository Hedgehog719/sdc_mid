#!/usr/bin/env python3
"""Extract image header capture times and match them exactly to receive-time image filenames."""
import argparse
import csv
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
        received = [int(name.name.split('_ts_')[1].split('.')[0]) for name in names]
        try:
            captured = np.array([timestamps[camera][timestamp] for timestamp in received], dtype=np.int64)
        except KeyError as error:
            raise ValueError(f'missing exact capture timestamp for {camera}: {error}') from error
        if np.any(np.diff(captured) <= 0):
            raise ValueError('capture timestamps must increase in frame order')
        output[camera] = captured
    return output


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
    arguments = parser.parse_args()
    output = arguments.out or arguments.track_dir / 'sensors' / 'camera_timing.csv'
    if not output.exists():
        extract(arguments.track_dir, output)
    arrays = capture_arrays(arguments.track_dir, metadata=output)
    for camera, captured in arrays.items():
        offset = (captured - arrays['f']) * 1e-6
        print(camera, 'matched frames', len(captured), 'capture offset ms p0/p50/p95/p100',
              np.percentile(offset, [0, 50, 95, 100]), flush=True)


if __name__ == '__main__':
    main()