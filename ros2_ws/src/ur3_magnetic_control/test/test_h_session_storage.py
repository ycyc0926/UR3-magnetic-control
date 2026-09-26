"""Exercise file writes with real frames and mocked ROS I/O; no ROS graph/hardware."""
import csv
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from rclpy.node import Node

from ur3_magnetic_control.h_tracker_node import HTrackerNode
from ur3_magnetic_control.experiment_data import active_experiment, current_experiment
from test_h_vision import frame


class SessionStorageTests(unittest.TestCase):
    def test_preview_writes_nothing_and_recording_can_stop_and_restart(self):
        with tempfile.TemporaryDirectory(prefix='h_session_test_') as directory:
            root = Path(directory)/'experiments'
            context_file = Path(directory)/'active'
            parameters = {'experiments_directory': str(root), 'web_preview': False}
            clock = Mock()
            with patch.multiple(
                Node, __init__=lambda *args: None,
                declare_parameter=lambda self, name, default: parameters.setdefault(name, default),
                get_parameter=lambda self, name: SimpleNamespace(value=parameters[name]),
                get_logger=lambda self: Mock(), get_clock=lambda self: clock,
                create_publisher=lambda *args: Mock(),
                create_subscription=lambda *args: None, create_timer=lambda *args: None,
                destroy_node=lambda self: None,
            ), patch('ur3_magnetic_control.h_tracker_node.current_experiment',
                     side_effect=lambda: current_experiment(context_file)):
                node = HTrackerNode()
                self.addCleanup(node.recorder.stop)
                node.web = SimpleNamespace(status=b'{}', jpeg=None, trajectory=None,
                                           pop_commands=lambda: [], close=lambda: None)

                def send(center=(320, 240)):
                    message = node.bridge.cv2_to_imgmsg(frame(center), encoding='bgr8')
                    stamp = time.time_ns()
                    clock.now.return_value.nanoseconds = stamp
                    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
                    node.on_image(message)

                self.assertFalse(root.exists())
                with self.assertRaises(RuntimeError):node.handle_command('record_start', 5)
                for _ in range(5):send()
                node.handle_command('clear', None)
                node.last_frame = time.monotonic()-1
                node.check_stream()
                self.assertFalse(root.exists())
                for _ in range(3):send()
                self.assertTrue(json.loads(node.web.status)['detected'])
                self.assertIsNotNone(node.web.jpeg)

                node.handle_command('record_start', 5)
                standalone = node.recorder.session
                self.assertEqual(standalone.parent.parent, root/'camera_only')
                self.assertEqual(standalone.name, 'h_robot')
                for x in range(320, 350):send((x, 240))
                node.handle_command('record_stop', None)
                self.assertFalse(node.recorder.active)
                self.assertTrue((node.recorder.directory/'unannotated.avi').is_file())
                self.assertFalse(json.loads(node.web.status)['recording']['active'])
                self.assertTrue(all(p.is_dir() and p.name.startswith('clip_')
                                    for p in standalone.iterdir()))
                self.assertEqual({p.name for p in node.recorder.directory.iterdir()},
                                 {'unannotated.avi', 'frame_times.csv', 'metadata.json', 'trajectory.png'})
                saved = {p: (p.stat().st_mtime_ns, p.read_bytes())
                         for p in root.rglob('*') if p.is_file()}
                for _ in range(5):send()
                node.handle_command('clear', None)
                node.update_status()
                self.assertEqual(saved, {p: (p.stat().st_mtime_ns, p.read_bytes())
                                         for p in root.rglob('*') if p.is_file()})

                experiment = root/'experiment_a'
                experiment.mkdir()
                with active_experiment(experiment, context_file):
                    for _ in range(3):send()
                    self.assertFalse((experiment/'h_robot').exists())
                    node.handle_command('record_start', 5)
                    self.assertEqual(node.recorder.session, experiment/'h_robot')
                    for x in range(320, 335):send((x, 240))
                next_experiment = root/'experiment_b'
                next_experiment.mkdir()
                with active_experiment(next_experiment, context_file):
                    with self.assertRaises(RuntimeError):node.handle_command('record_start', 5)
                    for x in range(335, 350):send((x, 240))
                    self.assertEqual(node.recorder.session, experiment/'h_robot')
                    self.assertFalse((next_experiment/'h_robot').exists())
                    node.handle_command('record_stop', None)
                    node.handle_command('record_start', 5)
                    self.assertEqual(node.recorder.session, next_experiment/'h_robot')
                    for x in range(350, 380):send((x, 240))
                node.recorder.deadline = time.monotonic()-1
                node.check_stream()
                self.assertFalse(node.recorder.active)
                self.assertEqual(len(list(root.glob('**/clip_*/unannotated.avi'))), 3)
                rows = []
                for path in root.glob('**/clip_*/frame_times.csv'):
                    with path.open() as stream:
                        rows.extend(csv.DictReader(stream))
                self.assertEqual(len(rows), 90)
                self.assertTrue(all(p.parent.name.startswith('clip_')
                                    for p in root.rglob('*') if p.is_file()))
                saved = {p: p.stat().st_mtime_ns for p in root.rglob('*') if p.is_file()}
                node.destroy_node()
                self.assertEqual(saved, {p: p.stat().st_mtime_ns for p in root.rglob('*') if p.is_file()})


if __name__ == '__main__':
    unittest.main()
