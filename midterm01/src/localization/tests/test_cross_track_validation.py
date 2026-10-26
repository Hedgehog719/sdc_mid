import os
import sys
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from validate_cross_track import evaluate_holdout


class HoldoutEvaluationTest(unittest.TestCase):
    def test_causal_extrapolation_uses_only_previous_pose(self):
        times = np.array([0, 1_000_000_000, 2_000_000_000])
        debug = pd.DataFrame({'x': [0.0, 2.0, 999.0], 'y': [0.0, 0.0, 999.0],
                              'yaw': [0.0, 0.0, 90.0], 'v': [2.0, 2.0, 0.0]})
        samples = pd.DataFrame({'t': [500_000_000, 1_500_000_000], 'x': [1.0, 3.0], 'y': [0.0, 0.0]})
        report = evaluate_holdout(times, debug, samples)
        self.assertEqual(report['heldout_samples'], 2)
        self.assertEqual(report['position_rmse'], 0.0)

    def test_heading_is_degrees_and_lateral_residual_is_separate(self):
        debug = pd.DataFrame({'x': [0.0, 0.0], 'y': [0.0, 2.0], 'yaw': [90.0, 90.0], 'v': [2.0, 2.0]})
        samples = pd.DataFrame({'t': [500_000_000], 'x': [0.2], 'y': [1.0]})
        report = evaluate_holdout([0, 1_000_000_000], debug, samples)
        self.assertAlmostEqual(report['forward_rmse'], 0.0)
        self.assertAlmostEqual(report['lateral_rmse'], 0.2)

    def test_empty_overlap_is_rejected(self):
        debug = pd.DataFrame({'x': [0.0], 'y': [0.0], 'yaw': [0.0], 'v': [0.0]})
        samples = pd.DataFrame({'t': [-1], 'x': [0.0], 'y': [0.0]})
        with self.assertRaises(ValueError):
            evaluate_holdout([0], debug, samples)


if __name__ == '__main__':
    unittest.main()