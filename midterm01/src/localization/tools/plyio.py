"""Minimal PLY reader (ascii / binary_little_endian, float xyz + uchar rgb)."""
import numpy as np


def read_ply_xyz(path):
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            header.append(line)
            if line == 'end_header':
                break
        n = int(next(h for h in header if h.startswith('element vertex')).split()[-1])
        props = [h.split() for h in header if h.startswith('property')]
        if 'ascii' in header[1]:
            data = np.loadtxt(f, max_rows=n, dtype=np.float64)
            return data[:, :3].astype(np.float32)
        dtype = np.dtype([(p[2], np.float32 if p[1] in ('float', 'float32') else np.uint8) for p in props])
        data = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype, count=n)
        return np.stack([data['x'], data['y'], data['z']], axis=1)
