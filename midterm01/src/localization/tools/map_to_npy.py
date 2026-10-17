#!/usr/bin/env python3
"""Convert the ASCII map.pcd into a full-resolution float32 .npy (x, y, z) cache. No downsampling."""
import argparse

import numpy as np
import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd')
    parser.add_argument('npy')
    args = parser.parse_args()
    skip = 0
    header = {}
    with open(args.pcd, 'rb') as f:
        for line in f:
            skip += 1
            key, *values = line.decode('ascii').split()
            header[key] = values
            if key == 'DATA':
                break
        if header['DATA'][0] == 'binary':
            types = {('F', '4'): np.float32, ('F', '8'): np.float64, ('U', '4'): np.uint32, ('I', '4'): np.int32,
                     ('U', '1'): np.uint8, ('U', '2'): np.uint16}
            dtype = np.dtype([(n, types[(t, s)]) for n, t, s in zip(header['FIELDS'], header['TYPE'], header['SIZE'])])
            data = np.fromfile(f, dtype=dtype, count=int(header['POINTS'][0]))
            xyz = np.stack([data['x'], data['y'], data['z']], axis=1).astype(np.float32)
            np.save(args.npy, xyz)
            if 'intensity' in data.dtype.names:
                np.save(args.npy.replace('.npy', '_intensity.npy'), data['intensity'].astype(np.float32))
            print(args.npy, xyz.shape, xyz.min(0), xyz.max(0))
            return
    chunks = []
    cols = header['FIELDS']
    use = [cols.index(c) for c in ['x', 'y', 'z']] + ([cols.index('intensity')] if 'intensity' in cols else [])
    for chunk in pd.read_csv(args.pcd, sep=' ', header=None, skiprows=skip, usecols=use,
                             dtype=np.float32, chunksize=5_000_000, engine='c'):
        chunks.append(chunk[use].values)
    data = np.concatenate(chunks)
    xyz = np.ascontiguousarray(data[:, :3])
    np.save(args.npy, xyz)
    if data.shape[1] > 3:
        np.save(args.npy.replace('.npy', '_intensity.npy'), data[:, 3].copy())
    print(args.npy, xyz.shape, xyz.min(0), xyz.max(0))


if __name__ == '__main__':
    main()
