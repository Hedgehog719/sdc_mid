import os
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from camera_timing import capture_arrays, read_capture_times


class CameraTimingTest(unittest.TestCase):
    def make_track(self, directory):
        root = Path(directory) / 'Track'
        (root / 'sensors').mkdir(parents=True)
        rows = ['camera,receive_ns,capture_ns']
        for camera, offset in [('f', 0), ('fl', 2), ('fr', -13)]:
            images = root / 'Track_images' / f'Track_{camera}_images'
            images.mkdir(parents=True)
            for index, received in enumerate([200, 300]):
                (images / f'{index:05d}_ts_{received}.jpg').touch()
                rows.append(f'{camera},{received},{received - 30 + offset}')
        (root / 'sensors' / 'camera_timing.csv').write_text('\n'.join(rows) + '\n')
        return root

    def test_exact_matching_preserves_integer_precision(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.make_track(directory)
            result = capture_arrays(root)
            np.testing.assert_array_equal(result['fr'] - result['f'], [-13, -13])
            np.testing.assert_array_equal(result['fl'] - result['f'], [2, 2])
            self.assertEqual(result['f'].dtype, np.int64)

    def test_missing_metadata_is_rejected_without_interpolation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.make_track(directory)
            (root / 'Track_images' / 'Track_fr_images' / '00002_ts_400.jpg').touch()
            with self.assertRaises(ValueError):
                capture_arrays(root)

    def test_invalid_receive_capture_order_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'times.csv'
            path.write_text('camera,receive_ns,capture_ns\nf,100,101\n')
            with self.assertRaises(ValueError):
                read_capture_times(path)


if __name__ == '__main__':
    unittest.main()