#!/usr/bin/env python3
"""Dump GNSS pose and IMU messages of a track bag to CSV (bag timestamps in ns)."""
import argparse
import csv
from pathlib import Path

from rosbags.highlevel import AnyReader


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('track_dir', help='e.g. /root/data/Track1')
    parser.add_argument('out_dir')
    args = parser.parse_args()
    track = Path(args.track_dir)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    with AnyReader([track]) as reader, \
            open(out / 'gnss.csv', 'w', newline='') as gf, open(out / 'imu.csv', 'w', newline='') as imf:
        g = csv.writer(gf)
        im = csv.writer(imf)
        g.writerow(['t', 'stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'frame'])
        im.writerow(['t', 'stamp', 'qx', 'qy', 'qz', 'qw', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'])
        conns = [c for c in reader.connections if c.topic in ('/localizer_node/gnss_pose', '/imu/data')]
        for conn, t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if conn.topic == '/imu/data':
                o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
                im.writerow([t, stamp, o.x, o.y, o.z, o.w, w.x, w.y, w.z, a.x, a.y, a.z])
            else:
                p, o = msg.pose.position, msg.pose.orientation
                g.writerow([t, stamp, p.x, p.y, p.z, o.x, o.y, o.z, o.w, msg.header.frame_id])


if __name__ == '__main__':
    main()
