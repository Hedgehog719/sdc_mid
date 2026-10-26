import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from camera_timing import capture_arrays, capture_offsets, read_capture_times, retime_vo_pairs
from bev import camera_time_offsets


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

    def test_side_only_offsets_still_reference_front_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            result = capture_offsets(self.make_track(directory), cameras=('fr',))
            self.assertEqual(set(result), {'fr'})
            np.testing.assert_allclose(result['fr'], [-13e-9, -13e-9], rtol=1e-12, atol=0)

    def test_synchronizer_capture_mode_is_opt_in(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.make_track(directory)
            with patch.dict(os.environ, {'SDC_CAM_CAPTURE': '0'}):
                previous = camera_time_offsets(str(root))
                np.testing.assert_array_equal(previous['fr'], [0, 0])
            with patch.dict(os.environ, {'SDC_CAM_CAPTURE': '1'}):
                actual = camera_time_offsets(str(root))
                np.testing.assert_allclose(actual['fr'], [-13e-9, -13e-9], rtol=1e-12, atol=0)
                np.testing.assert_array_equal(actual['f'], [0, 0])

    def test_different_camera_lengths_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.make_track(directory)
            (root / 'Track_images' / 'Track_fl_images' / '00001_ts_300.jpg').unlink()
            with self.assertRaises(ValueError):
                capture_offsets(root)

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


class VoCaptureTimingTest(unittest.TestCase):
    def sample(self):
        return {'j': np.array([0, 1, 2]), 'k': np.array([1, 2, 3]),
                'fwd': np.array([0.25, 0.26, 0.27]), 'lat': np.array([0.01, 0.0, -0.01]),
                'resp': np.array([0.9, 0.8, 0.9]), 'dt': np.array([0.01, 0.05, 0.03])}

    def test_only_timing_and_speed_change(self):
        original = self.sample()
        actual = retime_vo_pairs(original, np.array([100, 136, 172, 208]) * 1_000_000)
        for name in ['j', 'k', 'fwd', 'lat', 'resp']:
            np.testing.assert_array_equal(actual[name], original[name])
        np.testing.assert_allclose(actual['dt'], 0.036)
        np.testing.assert_allclose(actual['speed'], original['fwd'] / 0.036)
        np.testing.assert_array_equal(original['dt'], [0.01, 0.05, 0.03])

    def test_future_capture_time_cannot_change_earlier_pairs(self):
        first = retime_vo_pairs(self.sample(), [100, 136, 172, 208])
        later = retime_vo_pairs(self.sample(), [100, 136, 172, 300])
        np.testing.assert_array_equal(first['dt'][:2], later['dt'][:2])
        np.testing.assert_array_equal(first['speed'][:2], later['speed'][:2])

    def test_invalid_indices_and_intervals_are_rejected(self):
        for captured in [[100, 136, 136, 208], [100, 136, 172]]:
            with self.assertRaises(ValueError):
                retime_vo_pairs(self.sample(), captured)
        values = self.sample()
        values['k'][0] = 2
        with self.assertRaises(ValueError):
            retime_vo_pairs(values, [100, 136, 172, 208])

    def test_nonfinite_displacement_is_rejected(self):
        values = self.sample()
        values['fwd'][1] = np.nan
        with self.assertRaises(ValueError):
            retime_vo_pairs(values, [100, 136, 172, 208])


if __name__ == '__main__':
    unittest.main()