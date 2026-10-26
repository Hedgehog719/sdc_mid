import os
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import bev


class CameraSyncTest(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((bev.H, bev.W), np.float32)
        self.row, self.col = 120, bev.W // 2
        self.image[self.row, self.col] = 1.0

    def test_zero_rotation_preserves_translation(self):
        for distance in (-0.2, 0.0, 0.2):
            np.testing.assert_array_equal(bev.sync_motion(self.image, distance, 0.0),
                                          bev.shift_rows(self.image, distance))

    def test_rigid_motion_matches_world_coordinates(self):
        point = np.array([bev.X_MAX - (self.row + 0.5) * bev.RES,
                          bev.Y_MAX - (self.col + 0.5) * bev.RES])
        for angle in (-0.02, 0.02):
            distance = -0.15
            rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            expected = rotation @ point + distance * np.array([np.sin(angle), 1 - np.cos(angle)]) / angle
            warped = bev.sync_motion(self.image, distance, angle)
            rows, cols = np.indices(warped.shape)
            actual = np.array([bev.X_MAX - ((rows * warped).sum() / warped.sum() + 0.5) * bev.RES,
                               bev.Y_MAX - ((cols * warped).sum() / warped.sum() + 0.5) * bev.RES])
            np.testing.assert_allclose(actual, expected, atol=bev.RES * 0.05)

    def test_binary_mask_stays_binary(self):
        warped = bev.sync_motion(self.image.astype(np.uint8), -0.15, 0.02, cv2.INTER_NEAREST)
        self.assertTrue(np.isin(warped, [0, 1]).all())

    def test_current_frame_only(self):
        camera = bev.ImageBEV.__new__(bev.ImageBEV)
        camera.tshift = None
        camera.capture_dt = {'fr': np.array([-0.014, -0.014])}
        camera.set_motion_frame(1, 8.0, 0.2)
        self.assertIs(camera._sync('fr', 0, self.image), self.image)
        self.assertIs(camera._sync('f', 1, self.image), self.image)
        with patch.object(bev, 'CAM_SYNC_YAW', True):
            np.testing.assert_array_equal(camera._sync('fr', 1, self.image),
                                          bev.sync_motion(self.image, -0.112, -0.0028))
        with patch.object(bev, 'CAM_SYNC_YAW', False):
            np.testing.assert_array_equal(camera._sync('fr', 1, self.image), bev.shift_rows(self.image, -0.112))


if __name__ == '__main__':
    unittest.main()