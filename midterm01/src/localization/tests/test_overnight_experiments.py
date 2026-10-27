import csv
from pathlib import Path
import os
import sys
import subprocess
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from overnight_experiments import OFFSETS, snapshot_runner, validate_prefix


class CausalPrefixTest(unittest.TestCase):
    def test_runner_snapshot_does_not_follow_live_edits(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory)
            (package / 'tools').mkdir()
            live = package / 'tools' / 'run_online.sh'
            live.write_text("printf 'original runner\\n'\n")
            snapshot = snapshot_runner(package, package)
            live.write_text("printf 'changed runner\\n'\n")
            output = subprocess.check_output(['bash', str(snapshot)], text=True)
            self.assertEqual(output, 'original runner\n')

    def make_fixture(self, directory):
        directory = Path(directory)
        for offset in OFFSETS:
            with (directory / f'hyp_{offset}_dbg.csv').open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['k', 'll'])
                writer.writerows([[0, 0], [1, -abs(float(offset))]])
            with (directory / f'hyp_{offset}.csv').open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['idx', 'x', 'y', 'yaw'])
                writer.writerows([[0, 1, 2, 0.3], [1, 2, 3, 0.4]])
        (directory / 'full.csv').write_text('idx,x,y,yaw\n0,1,2,0.3\n1,2,3,0.4\n2,3,4,0.5\n')
        return directory

    def test_matching_prefix(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(validate_prefix(self.make_fixture(directory), 2), '0')

    def test_future_changed_prefix_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = self.make_fixture(directory)
            (directory / 'full.csv').write_text('idx,x,y,yaw\n0,9,2,0.3\n1,2,3,0.4\n')
            with self.assertRaises(ValueError):
                validate_prefix(directory, 2)

    def test_incomplete_hypothesis_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = self.make_fixture(directory)
            (directory / 'hyp_0_dbg.csv').write_text('k,ll\n0,0\n')
            with self.assertRaises(ValueError):
                validate_prefix(directory, 2)


if __name__ == '__main__':
    unittest.main()