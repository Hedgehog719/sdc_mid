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

    def test_capture_named_side_images_and_receive_named_front(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.make_track(directory)
            mapping = read_capture_times(root / 'sensors' / 'camera_timing.csv')
            for camera in ['fl', 'fr']:
                for image in (root / 'Track_images' / f'Track_{camera}_images').iterdir():
                    prefix, timestamp = image.stem.split('_ts_')
                    image.rename(image.with_name(f'{prefix}_ts_{mapping[camera][int(timestamp)]}.jpg'))
            arrays = capture_arrays(root)
            np.testing.assert_array_equal(arrays['fl'] - arrays['f'], [2, 2])
            np.testing.assert_array_equal(arrays['fr'] - arrays['f'], [-13, -13])

    def test_empty_camera_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.make_track(directory)
            for image in (root / 'Track_images' / 'Track_fr_images').iterdir():
                image.unlink()
            with self.assertRaises(ValueError):
                capture_arrays(root)

    def test_ambiguous_timestamp_convention_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.make_track(directory)
            metadata = root / 'sensors' / 'camera_timing.csv'
            metadata.write_text('camera,receive_ns,capture_ns\nf,200,100\nf,300,200\nf,400,300\n')
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