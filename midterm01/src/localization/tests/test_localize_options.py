import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


SOURCE = Path(__file__).resolve().parents[1] / 'tools' / 'localize_online.py'
TREE = ast.parse(SOURCE.read_text())


def load_function(name, environment):
    """Load pure CLI helpers without importing the map-initializing command-line module."""
    node = next(node for node in TREE.body if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), 'exec'), environment)
    return environment[name]


class StandstillBudgetTest(unittest.TestCase):
    def test_detector_dropout_preserves_budget_until_moving(self):
        node = next(node for node in ast.walk(TREE) if isinstance(node, ast.If)
                    and ast.unparse(node.test) == "D.get('still')" and 'still_k0' in ast.unparse(node))
        code = compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), 'exec')
        state = dict(D={'still': 1}, still_k0=None, still_used=0, k=1000, v=0.0)
        exec(code, state)
        self.assertEqual(state['still_k0'], 1000)
        state['still_used'] = 3
        for frame in [1093, 1107, 1115, 1121]:
            for detected in [False, True]:
                state.update(D={'still': int(detected)}, k=frame, v=0.015)
                exec(code, state)
                self.assertEqual((state['still_k0'], state['still_used']), (1000, 3))
        state.update(D={}, v=0.3)
        exec(code, state)
        self.assertIsNone(state['still_k0'])
        state.update(D={'still': 1}, v=0.0, k=3000)
        exec(code, state)
        self.assertEqual((state['still_k0'], state['still_used']), (3000, 0))


class GyroHeadingTest(unittest.TestCase):
    def setUp(self):
        self.rate = load_function('gyro_heading_rate', {'np': np})

    def test_body_roll_does_not_change_heading(self):
        orientation = Rotation.from_euler('ZYX', [[30, 15, 10]], degrees=True)
        rates = np.array([[0.02, 0.0, 0.0]])
        self.assertGreater(abs(orientation.apply(rates)[0, 2]), 0.001)
        np.testing.assert_allclose(self.rate(orientation, rates), 0.0, atol=1e-12)

    def test_world_yaw_rotation_has_correct_rate_at_tilt(self):
        orientation = Rotation.from_euler('ZYX', [[0, 15, 10], [45, -15, 20], [90, 5, -5]], degrees=True)
        body_rates = orientation.inv().apply(np.tile([0.0, 0.0, 0.1], (3, 1)))
        np.testing.assert_allclose(self.rate(orientation, body_rates), 0.1, atol=1e-12)

    def test_matches_quaternion_finite_difference(self):
        orientation = Rotation.from_euler('ZYX', [[20, -20, 15]], degrees=True)
        body_rates = np.array([[0.03, -0.02, 0.04]])
        interval = 1e-6
        following = orientation * Rotation.from_rotvec(body_rates * interval)
        difference = following.as_euler('ZYX')[:, 0] - orientation.as_euler('ZYX')[:, 0]
        expected = np.arctan2(np.sin(difference), np.cos(difference)) / interval
        np.testing.assert_allclose(self.rate(orientation, body_rates), expected, atol=1e-8)


class GnssSubsetTest(unittest.TestCase):
    def setUp(self):
        self.samples = pd.DataFrame({'t': [10, 20, 30, 40], 'x': [1.0, 2.0, 3.0, 4.0], 'y': [0.0] * 4})
        self.arguments = SimpleNamespace(track_dir='track', gnss=True, gnss_stride=1, gnss_phase=0)
        environment = {'a': self.arguments, 'np': np,
                       'os': SimpleNamespace(path=SimpleNamespace(join=lambda *parts: '/'.join(parts), exists=lambda path: True)),
                       'pd': SimpleNamespace(read_csv=lambda path: self.samples.copy())}
        self.load = load_function('load_gnss', environment)

    def test_default_is_unchanged(self):
        times, positions, _ = self.load()
        np.testing.assert_array_equal(times, self.samples.t)
        np.testing.assert_array_equal(positions, self.samples.x)

    def test_subsets_are_disjoint(self):
        self.arguments.gnss_stride = 2
        even = self.load()[0]
        self.arguments.gnss_phase = 1
        odd = self.load()[0]
        np.testing.assert_array_equal(even, [10, 30])
        np.testing.assert_array_equal(odd, [20, 40])
        self.assertFalse(set(even).intersection(odd))

    def test_invalid_subset_is_rejected(self):
        for stride, phase in [(0, 0), (2, 2), (2, -1)]:
            self.arguments.gnss_stride, self.arguments.gnss_phase = stride, phase
            with self.assertRaises(ValueError):
                self.load()


class InitialPositionTest(unittest.TestCase):
    def run_initial(self, offset, conditional):
        arguments = SimpleNamespace(init='', track_dir='track', seg_gate_track=True, seg_gate=0.5,
                                    init_yaw=25, lane_yaw_off=0.3, gross_yaw=5, init_pos=2.5,
                                    init_offset=offset, init_pos_per_hyp=conditional, min_peak_lo=0.13,
                                    seg_gate_all=False, seg_gate_from=300)
        matches = [(0.3, 1, 0, 0, np.radians(17.3)), (0.3, 1, 0, 0, np.radians(0.3)),
                   (0.3, 1, 1.0, 0.2, 0.0)]
        lane = SimpleNamespace(BM=SimpleNamespace(A=SimpleNamespace(seg_gate=0)), set_window=Mock(),
                               match=Mock(side_effect=matches))
        numeric = SimpleNamespace(loadtxt=lambda *args, **kwargs: np.eye(4), radians=np.radians,
                                  degrees=np.degrees, cos=np.cos, sin=np.sin)
        environment = {'a': arguments, 'np': numeric, 'Rotation': Rotation, 'LS': lane,
                       'os': SimpleNamespace(path=SimpleNamespace(join=lambda *parts: '/'.join(parts)))}
        pose = load_function('initial_pose', environment)(None)
        return np.array(pose), lane.match.call_args.args[1][2]

    def test_zero_offset_is_unchanged(self):
        baseline, _ = self.run_initial(0.0, False)
        conditional, _ = self.run_initial(0.0, True)
        np.testing.assert_array_equal(baseline, conditional)

    def test_position_uses_hypothesis_heading_without_double_offset(self):
        pose, position_heading = self.run_initial(-1.5, True)
        expected = np.radians(15.5)
        self.assertAlmostEqual(position_heading, expected)
        self.assertAlmostEqual(pose[0], np.cos(expected) - 0.2 * np.sin(expected))
        self.assertAlmostEqual(pose[1], np.sin(expected) + 0.2 * np.cos(expected))
        self.assertAlmostEqual(pose[2], np.radians(17.0))


if __name__ == '__main__':
    unittest.main()