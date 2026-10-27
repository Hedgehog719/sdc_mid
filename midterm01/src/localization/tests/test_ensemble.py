import os
import sys
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from ensemble import combine_runs


class EnsembleTest(unittest.TestCase):
    def sample(self, heading=0.0):
        return pd.DataFrame({'idx': [0, 1, 2], 'x': [0.0, 1.0, 2.0], 'y': [0.0] * 3, 'yaw': [heading] * 3})

    def test_heading_wrap_and_equal_position_weights(self):
        first, second = self.sample(np.radians(179)), self.sample(np.radians(-179))
        second.x += 2
        result = combine_runs([first, second])
        np.testing.assert_allclose(result.x, [1, 2, 3])
        np.testing.assert_allclose(np.abs(result.yaw), np.pi)

    def test_future_rows_cannot_change_prefix(self):
        first, second = self.sample(0.1), self.sample(0.2)
        full = combine_runs([first, second])
        prefix = combine_runs([first.iloc[:2], second.iloc[:2]])
        np.testing.assert_array_equal(full.iloc[:2].to_numpy(), prefix.to_numpy())

    def test_misaligned_indices_are_rejected(self):
        first, second = self.sample(), self.sample()
        second.idx += 1
        with self.assertRaises(ValueError):
            combine_runs([first, second])

    def test_missing_and_nonfinite_members_are_rejected(self):
        with self.assertRaises(ValueError):
            combine_runs([])
        frame = self.sample()
        with self.assertRaises(ValueError):
            combine_runs([frame, frame.iloc[:2]])
        frame.loc[1, 'x'] = np.nan
        with self.assertRaises(ValueError):
            combine_runs([frame])


if __name__ == '__main__':
    unittest.main()