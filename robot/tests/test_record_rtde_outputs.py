import time
import unittest
from record_rtde_outputs import OutputRecorder


class RecorderTests(unittest.TestCase):
    def test_missing_and_stale_records_fail(self):
        recorder=OutputRecorder('/tmp/NOT_OPENED_BY_THIS_TEST')
        with self.assertRaisesRegex(RuntimeError,'stale'):recorder.check_fresh(.12)
        recorder.latest={'host_monotonic_s':time.monotonic()}
        recorder.check_fresh(.12)
        recorder.latest['host_monotonic_s']-=1.
        with self.assertRaisesRegex(RuntimeError,'stale'):recorder.check_fresh(.12)

    def test_recorder_error_is_not_hidden_by_a_fresh_record(self):
        recorder=OutputRecorder('/tmp/NOT_OPENED_BY_THIS_TEST')
        recorder.latest={'host_monotonic_s':time.monotonic()};recorder.error='output stream lost'
        with self.assertRaisesRegex(RuntimeError,'output stream lost'):recorder.check_fresh(.12)


if __name__=='__main__':unittest.main()
