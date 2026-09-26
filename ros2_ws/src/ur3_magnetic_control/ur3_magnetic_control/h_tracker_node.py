"""H-target preview and recordings. Publishes observations only, never motion."""
from collections import deque
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from .h_tracking import DarkTargetTracker, PlaneMapper
from .h_preview_web import LocalPreview
from .h_recording import BoundedRecorder
from .experiment_data import current_experiment


class HTrackerNode(Node):
    def __init__(self):
        super().__init__('h_tracker')
        self.declare_parameter('calibration_file', '/home/yc/UR3/config/camera_robot_calibration.yaml')
        self.declare_parameter('experiments_directory', '/home/yc/UR3/experiments')
        self.declare_parameter('show_window', False)
        self.declare_parameter('web_preview', True)
        self.declare_parameter('reacquire_frames', 3)
        self.declare_parameter('recording_movement_threshold_mm', 0.5)
        self.declare_parameter('recording_minimum_detection_fraction', 0.8)
        self.mapper = PlaneMapper(self.get_parameter('calibration_file').value)
        self.detector = DarkTargetTracker(
            reacquire_frames=self.get_parameter('reacquire_frames').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.web = LocalPreview() if self.get_parameter('web_preview').value else None
        self.bridge = CvBridge()
        self.experiments_directory = Path(self.get_parameter('experiments_directory').value)
        self.recorder = BoundedRecorder(
            self.experiments_directory,
            movement_threshold_mm=float(
                self.get_parameter('recording_movement_threshold_mm').value),
            minimum_detection_fraction=float(
                self.get_parameter('recording_minimum_detection_fraction').value))
        self.last_recording_finalization = 0
        self.command_error = None
        self.pose_pub = self.create_publisher(PoseStamped, '/h_robot/pose_world', 1)
        self.pixel_pub = self.create_publisher(PoseStamped, '/h_robot/pose_pixels', 1)
        self.detected_pub = self.create_publisher(Bool, '/h_robot/detected', 1)
        self.score_pub = self.create_publisher(Float32, '/h_robot/tracking_score', 1)
        self.preview_pub = self.create_publisher(Image, '/h_robot/image_annotated', 1)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_timer(.1, self.check_stream)
        self.trail = deque(maxlen=200)
        self.samples = deque(maxlen=400)
        self.arrival_times = deque(maxlen=50)
        self.last_frame = None
        self.frames = 0
        self.successes = 0
        self.consecutive = 0
        self.origin_xy = None
        self.latest = None
        self.title = 'H Robot - Vision Only (R: reacquire, C: clear trail, Q: close)'
        if self.show_window:
            cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.title, 1100, 920)
            cv2.setMouseCallback(self.title, self.on_click)
        self.get_logger().info('Vision only; files are saved only during requested recordings')

    def on_click(self, event, x, y, flags, parameter):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.reset((x, y))

    def reset(self, seed=None):
        self.recorder.mark_tracking_discontinuity()
        self.detector.reset(seed)
        self.trail.clear(); self.samples.clear()
        self.origin_xy = None; self.consecutive = 0

    def handle_command(self, command, value):
        self.command_error = None
        if command in ('reset', 'select'):
            self.reset(value)
        elif command == 'clear':
            self.trail.clear(); self.samples.clear(); self.origin_xy = None
        elif command == 'record_start':
            if self.last_frame is None or time.monotonic()-self.last_frame > .25:
                raise RuntimeError('相机无新图像，未开始录像')
            experiment = current_experiment()
            if experiment is None:
                experiment = self.experiments_directory/'camera_only'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.recorder.start(value, session=experiment/'h_robot')
            if self.web:
                self.web.trajectory = None
        elif command == 'record_stop':
            self.recorder.stop()
            self.sync_recording_result()

    def sync_recording_result(self):
        if self.recorder.finalization_serial == self.last_recording_finalization:
            return
        status = self.recorder.status()
        self.last_recording_finalization = self.recorder.finalization_serial
        if self.web:
            self.web.trajectory = None
            trajectory = status.get('trajectory_path')
            if trajectory:
                try:
                    self.web.trajectory = Path(trajectory).read_bytes()
                except OSError as error:
                    self.get_logger().warning(f'Could not load trajectory preview: {error}')
        if self.latest is not None:
            self.latest['recording'] = status
            self.update_status()

    def publish_invalid(self):
        self.detected_pub.publish(Bool(data=False))
        self.score_pub.publish(Float32(data=0.0))
        self.consecutive = 0
        self.trail.clear()  # Never join a trail across missing observations.

    def check_stream(self):
        if self.web:
            for command, value in self.web.pop_commands():
                try:
                    self.handle_command(command, value)
                except (ValueError, RuntimeError, OSError) as error:
                    self.command_error = str(error)
        self.recorder.tick()
        self.sync_recording_result()
        if self.latest is not None:
            self.latest.update(recording=self.recorder.status(), command_error=self.command_error)
        if self.last_frame is None or time.monotonic()-self.last_frame > .25:
            self.publish_invalid()
            self.detector.mark_lost()
            if self.latest is not None:
                self.latest['detected'] = False
                self.latest['stream_stale'] = True
                self.latest['tracking_reason'] = 'stream_timeout'
                self.update_status()
        if self.show_window:
            key = cv2.waitKey(1) & 0xff
            if key == ord('r'):
                self.reset()
            elif key == ord('c'):
                self.trail.clear();self.samples.clear();self.origin_xy = None
            elif key == ord('q') or cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) < 1:
                rclpy.shutdown()

    def update_status(self):
        if self.web:self.web.status = json.dumps(self.latest).encode()

    def on_image(self, message):
        now = time.monotonic()
        age = (self.get_clock().now().nanoseconds-
               (message.header.stamp.sec*10**9+message.header.stamp.nanosec))/1e9
        self.last_frame = now
        self.arrival_times.append(now)
        self.frames += 1
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        # Normalize to half sensor resolution so area/jump thresholds are constant.
        if image.shape[:2] != (1024, 1224):
            image = cv2.resize(image, (1224, 1024), interpolation=cv2.INTER_AREA)
        fresh = -.1 <= age <= .2
        if fresh:
            result = self.detector.detect(image)
            diagnostic = self.detector.diagnostics
        else:
            result = None
            self.detector.mark_lost()
            diagnostic = {'reason': 'image_timestamp_stale'}
        preview = image.copy()
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        reason = diagnostic['reason']
        xy = full_pixel = None
        if result is not None:
            xy, full_pixel = self.mapper.project(result['pixel'], (1224, 1024))
        if reason == 'reacquired':
            self.recorder.mark_tracking_discontinuity()
            self.samples.clear()
        self.recorder.write(image, stamp, result is not None, reason,
                            None if xy is None else xy*1000)
        self.sync_recording_result()
        self.latest = {'utc': datetime.now(timezone.utc).isoformat(), 'host_frame_time_ns': stamp,
                       'detected': result is not None, 'frames': self.frames,
                       'stream_stale': not fresh, 'world_mapping_verified': False,
                       'tracking_reason': reason,
                       'tracking_diagnostics': diagnostic, 'recording': self.recorder.status(),
                       'command_error': self.command_error}
        lines = []
        if result is None:
            self.publish_invalid()
            lines += [f'TARGET LOST ({reason}) - auto reacquiring; click H to select']
        else:
            self.successes += 1; self.consecutive += 1
            self.samples.append(xy*1000)
            if self.origin_xy is None:
                self.origin_xy = xy.copy()
            self.trail.append(tuple(np.rint(result['pixel']).astype(int)))
            for left, right in zip(list(self.trail), list(self.trail)[1:]):
                cv2.line(preview, left, right, (0, 180, 255), 1)
            cv2.drawContours(preview, [result['contour']], -1, (0, 255, 0), 1)
            x, y, w, h = result['box']
            cv2.rectangle(preview, (x-8, y-8), (x+w+8, y+h+8), (0, 220, 0), 2)
            cv2.drawMarker(preview, self.trail[-1], (0, 0, 255), cv2.MARKER_CROSS, 15, 1)
            pose = PoseStamped();pose.header = copy.deepcopy(message.header);pose.header.frame_id = 'table_world'
            pose.pose.position.x, pose.pose.position.y = (float(v) for v in xy)
            pose.pose.position.z = self.mapper.z
            pose.pose.orientation.w = 1.0  # Placeholder; orientation is not measured.
            self.pose_pub.publish(pose)
            pixel = PoseStamped();pixel.header = copy.deepcopy(message.header);pixel.header.frame_id = 'camera_full_resolution_pixels'
            pixel.pose.position.x, pixel.pose.position.y = (float(v) for v in full_pixel)
            pixel.pose.orientation.w = 1.0
            self.pixel_pub.publish(pixel)
            self.detected_pub.publish(Bool(data=True))
            self.score_pub.publish(Float32(data=float(result['confidence'])))
            spread = np.std(np.asarray(self.samples), axis=0)
            delta = (xy-self.origin_xy)*1000
            self.latest.update({'pixel_full': full_pixel.tolist(), 'world_xy_mm': (xy*1000).tolist(),
                                'window_std_mm': spread.tolist(), 'window_samples': len(self.samples),
                                'consecutive_detections': self.consecutive, 'area_half_px': result['area_px'],
                                'displacement_from_reset_mm': delta.tolist()})
            lines += [f'H: X={xy[0]*1000:.2f} mm  Y={xy[1]*1000:.2f} mm']
        fps = (len(self.arrival_times)-1)/(self.arrival_times[-1]-self.arrival_times[0]) if len(self.arrival_times)>1 else 0
        self.latest['fps'] = fps
        self.latest['detection_fraction'] = self.successes/self.frames
        record = self.recorder.status()
        record_text = f'RECORDING {record["frames"]} frames, {record["seconds_remaining"]:.0f}s left' if record['active'] else 'VIDEO OFF'
        lines += [f'{fps:.1f} FPS | {record_text}']
        if self.show_window:
            lines += ['R=reacquire  C=clear trail  Q=close']
        cv2.rectangle(preview, (0, 0), (1224, len(lines)*28+12), (32, 32, 32), -1)
        for i, line in enumerate(lines):
            cv2.putText(preview, line, (12, 27+i*28), cv2.FONT_HERSHEY_SIMPLEX, .59, (230, 230, 230), 1, cv2.LINE_AA)
        msg = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8');msg.header = message.header
        self.preview_pub.publish(msg)
        if self.show_window:
            cv2.imshow(self.title, preview)
        if self.web:
            self.update_status()
            if self.frames % 4 == 0:
                self.web.jpeg = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()

    def destroy_node(self):
        if self.latest is not None:
            self.latest['detected'] = False
            self.latest['viewer_closed'] = True
        self.recorder.stop('viewer_closed')
        self.sync_recording_result()
        if self.latest is not None:
            self.latest['recording'] = self.recorder.status()
            self.update_status()
        if self.show_window:
            cv2.destroyAllWindows()
        if self.web:self.web.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node = HTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
