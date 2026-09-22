"""Offline tests only: no ROS graph, robot connection, motor, or camera device."""
import csv
import json
from pathlib import Path
import tempfile
import time
import unittest
import urllib.error
import urllib.request

import cv2
import numpy as np

from ur3_magnetic_control.h_preview_web import LocalPreview, PAGE
from ur3_magnetic_control.h_recording import BoundedRecorder
from ur3_magnetic_control.h_tracking import DarkTargetTracker, PlaneMapper, reference_path


ROOT = Path(__file__).resolve().parents[4]


def frame(center=(320, 240)):
    image = np.full((480, 640, 3), 220, np.uint8)
    x, y = center
    cv2.rectangle(image, (x-18, y-25), (x-8, y+25), (20, 20, 20), -1)
    cv2.rectangle(image, (x+8, y-25), (x+18, y+25), (20, 20, 20), -1)
    cv2.rectangle(image, (x-10, y-5), (x+10, y+5), (20, 20, 20), -1)
    return image


class ProjectionTests(unittest.TestCase):
    def setUp(self):
        self.mapper = PlaneMapper(ROOT/'config/camera_robot_calibration.yaml')

    def test_round_trip(self):
        for pixel in [(600, 500), (735, 875), (820, 610), (300, 350)]:
            xy, _ = self.mapper.project(pixel, (1224, 1024))
            back = self.mapper.image_points([xy], (1224, 1024))[0]
            np.testing.assert_allclose(back, pixel, atol=.01)

    def test_sensor_scaling(self):
        xy, full = self.mapper.project((700, 800), (1224, 1024))
        actual, _ = self.mapper.project(full, (2448, 2048))
        np.testing.assert_allclose(xy, actual, atol=1e-12)

    def test_reference_square_and_line(self):
        start = [.2, .1]
        square = reference_path('square', start, 20)
        np.testing.assert_allclose(square[-1], start)
        np.testing.assert_allclose(np.linalg.norm(np.diff(square, axis=0), axis=1), .02)
        np.testing.assert_allclose(square[1]-square[0], [0, .02])
        self.assertEqual(reference_path('line', start, 10).shape, (2, 2))
        np.testing.assert_allclose(reference_path('line_negative_y', start, 10)[1]-start, [0, -.01])

    def test_bad_reference_rejected(self):
        for size in [float('nan'), 0, 100]:
            with self.assertRaises(ValueError): reference_path('square', [0, 0], size)
        with self.assertRaises(ValueError):reference_path('robot_move', [0, 0], 20)


class TrackerTests(unittest.TestCase):
    def test_diagnostics_and_lockout_unchanged(self):
        tracker = DarkTargetTracker()
        self.assertIsNotNone(tracker.detect(frame()))
        self.assertEqual(tracker.diagnostics['reason'], 'tracked')
        blank = np.full((480, 640, 3), 220, np.uint8)
        for i in range(5):self.assertIsNone(tracker.detect(blank))
        self.assertTrue(tracker.locked_out)
        self.assertIsNone(tracker.detect(frame()))
        self.assertEqual(tracker.diagnostics['reason'], 'locked_out')
        tracker.reset((320, 240))
        self.assertIsNotNone(tracker.detect(frame()))

    def test_jump_is_rejected_and_recorded(self):
        tracker = DarkTargetTracker()
        tracker.detect(frame())
        self.assertIsNone(tracker.detect(frame((400, 240))))
        self.assertGreater(tracker.diagnostics['rejected']['distance'], 0)


class RecorderTests(unittest.TestCase):
    def test_records_invalid_detections_and_real_timestamps(self):
        with tempfile.TemporaryDirectory(prefix='h_vision_test_') as directory:
            recorder = BoundedRecorder(directory)
            recorder.start(5)
            for i in range(12):
                recorder.write(frame(), 1000000000+i*50000000, i < 3, 'tracked' if i < 3 else 'locked_out')
            recorder.stop()
            clip = recorder.directory
            with (clip/'frame_times.csv').open() as stream:rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 12)
            self.assertEqual(sum(int(r['detected']) for r in rows), 3)
            self.assertEqual(int(rows[-1]['host_frame_time_ns']), 1550000000)
            capture = cv2.VideoCapture(str(clip/'unannotated.avi'))
            count = 0
            while capture.read()[0]:count += 1
            capture.release()
            self.assertEqual(count, 12)
            metadata = json.loads((clip/'metadata.json').read_text())
            self.assertFalse(metadata['controls_motor'])
            self.assertFalse(metadata['active'])
            self.assertAlmostEqual(metadata['max_received_frame_gap_s'], .05)
            self.assertEqual(metadata['classification'], 'indeterminate')
            self.assertTrue(metadata['kept'])

    def test_moving_center_keeps_video_and_draws_trajectory(self):
        with tempfile.TemporaryDirectory(prefix='h_vision_test_') as directory:
            recorder = BoundedRecorder(directory, reserve_bytes=0)
            recorder.start(5)
            for index in range(30):
                recorder.write(frame(), 10**9+index*50_000_000, True, 'tracked',
                               [100+index*.1, 80+index*.03])
            recorder.stop()
            status = recorder.status()
            self.assertEqual(status['classification'], 'moving')
            self.assertTrue(status['kept'])
            self.assertGreater(status['motion']['center_span_mm'], 2.5)
            self.assertTrue((recorder.directory/'unannotated.avi').is_file())
            self.assertTrue((recorder.directory/'trajectory.png').is_file())
            self.assertTrue(status['trajectory_available'])
            with (recorder.directory/'frame_times.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertAlmostEqual(float(rows[-1]['world_x_mm']), 102.9)

    def test_static_center_is_discarded_after_recording(self):
        with tempfile.TemporaryDirectory(prefix='h_vision_test_') as directory:
            recorder = BoundedRecorder(directory, reserve_bytes=0)
            recorder.start(5)
            clip = recorder.directory
            for index in range(30):
                jitter = .02*np.sin(index)
                recorder.write(frame(), 10**9+index*50_000_000, True, 'tracked',
                               [100+jitter, 80-jitter])
            recorder.stop()
            status = recorder.status()
            self.assertEqual(status['classification'], 'static')
            self.assertFalse(status['kept'])
            self.assertEqual(status['reason'], 'discarded_no_center_motion')
            self.assertIsNone(recorder.directory)
            self.assertFalse(clip.exists())
            self.assertLess(status['motion']['center_span_mm'], .5)

    def test_single_center_outlier_does_not_keep_static_video(self):
        with tempfile.TemporaryDirectory(prefix='h_vision_test_') as directory:
            recorder = BoundedRecorder(directory, reserve_bytes=0)
            recorder.start(5)
            for index in range(30):
                position = [105, 81] if index == 15 else [100, 80]
                recorder.write(frame(), 10**9+index*50_000_000, True, 'tracked', position)
            recorder.stop()
            self.assertEqual(recorder.status()['classification'], 'static')

    def test_tracking_gap_is_kept_instead_of_risking_false_static_deletion(self):
        with tempfile.TemporaryDirectory(prefix='h_vision_test_') as directory:
            recorder = BoundedRecorder(directory, reserve_bytes=0)
            recorder.start(5)
            for index in range(30):
                detected = index < 10
                recorder.write(frame(), 10**9+index*50_000_000, detected,
                               'tracked' if detected else 'locked_out',
                               [100, 80] if detected else None)
            recorder.stop()
            status = recorder.status()
            self.assertEqual(status['classification'], 'indeterminate')
            self.assertTrue(status['kept'])
            self.assertTrue(recorder.directory.exists())

    def test_deadline_ends_only_recording(self):
        with tempfile.TemporaryDirectory(prefix='h_vision_test_') as directory:
            recorder = BoundedRecorder(directory)
            recorder.start(5)
            recorder.deadline = time.monotonic()-1
            recorder.tick()
            self.assertFalse(recorder.active)
            self.assertEqual(recorder.reason, 'recording_time_limit')
            self.assertFalse(recorder.status()['motor_control'])

    def test_bounds_and_duplicate_start(self):
        with tempfile.TemporaryDirectory(prefix='h_vision_test_') as directory:
            recorder = BoundedRecorder(directory)
            for duration in [0, 1000, float('nan')]:
                with self.assertRaises(ValueError): recorder.start(duration)
            recorder.start(5)
            with self.assertRaises(RuntimeError):recorder.start(5)
            recorder.stop()

    def test_dimension_change_is_reported(self):
        with tempfile.TemporaryDirectory(prefix='h_vision_test_') as directory:
            recorder = BoundedRecorder(directory)
            recorder.start(5)
            recorder.write(frame(), 1, True, 'tracked')
            recorder.write(frame()[:100, :100], 2, True, 'tracked')
            self.assertFalse(recorder.active)
            self.assertEqual(recorder.reason, 'recording_error')
            self.assertIn('size changed', recorder.last_error)


class WebTests(unittest.TestCase):
    def test_page_has_no_embedded_arm_control(self):
        for fragment in ('arm-control', 'arm-panel', 'arm-waiting-button', 'ARM_PANEL_URL'):
            self.assertNotIn(fragment, PAGE)

    def test_commands_are_validated_and_queued(self):
        web = LocalPreview(port=0)
        port = web.server.server_address[1]
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        def post(path, body, origin=None):
            headers = {'Content-Type': 'application/json'}
            if origin is not None:headers['Origin'] = origin
            req = urllib.request.Request(f'http://127.0.0.1:{port}'+path, json.dumps(body).encode(), headers, method='POST')
            return opener.open(req, timeout=2)
        try:
            post('/record/start', {'duration_s': 5}).close()
            post('/event', {'label': 'motor_started'}).close()
            post('/event', {'label': 'motor_stopped'}).close()
            post('/record/stop', {}).close()
            self.assertEqual([c[0] for c in web.pop_commands()], ['record_start', 'event', 'event', 'record_stop'])
            post('/path', {'kind': 'line_negative_y', 'size_mm': 10}, f'http://127.0.0.1:{port}').close()
            self.assertEqual(list(web.pop_commands()), [('path', ('line_negative_y', 10))])
            for path, body in [('/record/start', {'duration_s': 999}),
                               ('/event', {'label': 'execute'}), ('/select', {'x': float('nan'), 'y': 0}),
                               ('/path', {'kind': 'square', 'size_mm': float('nan')}), ('/reset', [])]:
                with self.assertRaises(urllib.error.HTTPError) as result:post(path, body)
                self.assertEqual(result.exception.code, 400)
            with self.assertRaises(urllib.error.HTTPError) as result:post('/reset', {}, 'https://example.com')
            self.assertEqual(result.exception.code, 403)
        finally:
            web.close()


if __name__ == '__main__':
    unittest.main()
