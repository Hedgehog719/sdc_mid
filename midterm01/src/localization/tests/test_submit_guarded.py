from datetime import datetime
import os
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from submit_guarded import check_deadline, validate_csv


class SubmissionGuardTest(unittest.TestCase):
    deadline = datetime.fromisoformat('2026-10-27T08:00:00+08:00')

    def test_before_cutoff(self):
        check_deadline(datetime.fromisoformat('2026-10-26T23:57:59+00:00'), self.deadline)

    def test_cutoff_and_after_deadline_are_rejected(self):
        for current in ['2026-10-27T07:58:00+08:00', '2026-10-27T08:00:00+08:00',
                        '2026-10-27T09:00:00+08:00']:
            with self.assertRaises(ValueError):
                check_deadline(datetime.fromisoformat(current), self.deadline)

    def test_naive_time_and_negative_buffer_are_rejected(self):
        with self.assertRaises(ValueError):
            check_deadline(datetime(2026, 10, 27, 7), self.deadline)
        with self.assertRaises(ValueError):
            check_deadline(self.deadline, self.deadline, -1)

    def test_valid_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'submission.csv'
            path.write_text('idx,x,y,yaw\n0,1,2,0\n1,3,4,3.14\n')
            validate_csv(path, 2)

    def test_invalid_poses_and_indices_are_rejected(self):
        for content in ['idx,x,y,yaw\n1,1,2,0\n', 'idx,x,y,yaw\n0,nan,2,0\n',
                        'idx,x,y,yaw\n0,1,2,4\n', 'idx,x,y,yaw\n', 'x,y,yaw\n']:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'submission.csv'
                path.write_text(content)
                with self.assertRaises(ValueError):
                    validate_csv(path, 1)


if __name__ == '__main__':
    unittest.main()