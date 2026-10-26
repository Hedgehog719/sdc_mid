import os
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from gnss_heading import HeadingPrior, quaternion_headings


class GnssHeadingTest(unittest.TestCase):
    def test_quaternions_normalize_and_invalid_samples_stay_invalid(self):
        quaternion = Rotation.from_euler('z', np.radians(179)).as_quat()
        result = quaternion_headings([quaternion * 2, [0, 0, 0, 0], [np.nan, 0, 0, 1]])
        self.assertAlmostEqual(result[0], np.radians(179))
        self.assertTrue(np.isnan(result[1:]).all())

    def test_first_future_sample_cannot_rewrite_frame_zero(self):
        prior = HeadingPrior()
        self.assertIsNone(prior.consume(100, 0, 0.2))
        self.assertIsNone(prior.last_time)
        self.assertEqual(prior.consume(100, 100, 0.2), 0.2)

    def test_stale_and_invalid_samples_do_not_consume_once_budget(self):
        prior = HeadingPrior(max_age_ns=50)
        self.assertIsNone(prior.consume(0, 51, 0.2))
        self.assertIsNone(prior.consume(60, 60, np.nan))
        self.assertEqual(prior.consume(70, 70, 0.3), 0.3)

    def test_once_prior_is_used_only_once(self):
        prior = HeadingPrior()
        self.assertEqual(prior.consume(0, 0, 0.2), 0.2)
        self.assertIsNone(prior.consume(3_000_000_000, 3_000_000_000, 0.4))

    def test_continuous_prior_rejects_duplicates_and_short_intervals(self):
        prior = HeadingPrior(once=False, min_interval_ns=100, max_age_ns=50)
        self.assertEqual(prior.consume(100, 100, 0.2), 0.2)
        self.assertIsNone(prior.consume(100, 100, 0.3))
        self.assertIsNone(prior.consume(199, 199, 0.3))
        self.assertIsNone(prior.consume(200, 200, 0.2))
        self.assertEqual(prior.consume(201, 201, 0.3), 0.3)

    def test_heading_wrap_does_not_create_a_new_measurement(self):
        prior = HeadingPrior(once=False, min_interval_ns=0)
        self.assertAlmostEqual(prior.consume(0, 0, np.pi), np.pi)
        self.assertIsNone(prior.consume(1, 1, -np.pi))


if __name__ == '__main__':
    unittest.main()