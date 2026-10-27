import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from hyp_select import posterior_mean


class PosteriorMeanTest(unittest.TestCase):
    def test_equal_evidence_averages_positions(self):
        poses = np.array([[[0.0, 2.0, 0.0]], [[2.0, 4.0, 0.0]]])
        actual = posterior_mean(np.zeros((2, 1)), [-1.0, 1.0], poses)
        np.testing.assert_allclose(actual, [[1.0, 3.0, 0.0]])

    def test_heading_wrap(self):
        poses = np.array([[[0.0, 0.0, np.radians(179)]], [[0.0, 0.0, np.radians(-179)]]])
        actual = posterior_mean(np.zeros((2, 1)), [-1.0, 1.0], poses)
        self.assertAlmostEqual(abs(actual[0, 2]), np.pi)

    def test_concentrated_evidence_selects_winner_without_overflow(self):
        poses = np.array([[[1.0, 2.0, 0.3]], [[9.0, 8.0, -0.2]]])
        actual = posterior_mean(np.array([[1e6], [-1e6]]), [0.0, 1.5], poses)
        np.testing.assert_allclose(actual, poses[0])

    def test_future_evidence_cannot_change_prefix(self):
        rng = np.random.default_rng(123)
        evidence = rng.normal(size=(5, 30))
        poses = rng.normal(size=(5, 30, 3))
        offsets = [0.0, -1.5, 1.5, -3.0, 3.0]
        short = posterior_mean(evidence[:, :10], offsets, poses[:, :10])
        full = posterior_mean(evidence, offsets, poses)
        np.testing.assert_array_equal(short, full[:10])


if __name__ == '__main__':
    unittest.main()