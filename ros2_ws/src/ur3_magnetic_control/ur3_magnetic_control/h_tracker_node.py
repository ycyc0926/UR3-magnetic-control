"""H-target preview and recordings. Publishes observations only, never motion."""
from collections import deque
import copy
import csv
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

from .h_tracking import DarkTargetTracker, PlaneMapper, reference_path
from .h_preview_web import LocalPreview
from .h_recording import BoundedRecorder


class HTrackerNode(Node):
    def __init__(self):
        super().__init__('h_tracker')
        self.declare_parameter('calibration_file', '/home/yc/UR3/config/camera_robot_calibration.yaml')
        self.declare_parameter('sessions_directory', '/home/yc/UR3/camera/tracking_sessions')
        self.declare_parameter('show_window', False)
        self.declare_parameter('web_preview', True)
        self.declare_parameter('recording_movement_threshold_mm', 0.5)
        self.declare_parameter('recording_minimum_detection_fraction', 0.8)
        self.mapper = PlaneMapper(self.get_parameter('calibration_file').value)
        self.detector = DarkTargetTracker()
        self.show_window = bool(self.get_parameter('show_window').value)
        self.web = LocalPreview() if self.get_parameter('web_preview').value else None
        self.bridge = CvBridge()
        self.session = Path(self.get_parameter('sessions_directory').value) / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.session.mkdir(parents=True, exist_ok=False)
        self.log = (self.session/'positions.csv').open('x', newline='', encoding='utf-8')
        self.writer = csv.writer(self.log)
        self.writer.writerow(['host_frame_time_ns', 'detected', 'u_full_px', 'v_full_px',
                              'world_x_mm', 'world_y_mm', 'area_half_px', 'tracking_score'])
        self.events = (self.session/'events.jsonl').open('x', encoding='utf-8')
        self.diagnostics_log = (self.session/'diagnostics.jsonl').open('x', encoding='utf-8')
        self.recorder = BoundedRecorder(
            self.session,
            movement_threshold_mm=float(
                self.get_parameter('recording_movement_threshold_mm').value),
            minimum_detection_fraction=float(
                self.get_parameter('recording_minimum_detection_fraction').value))
        self.last_recording_finalization = 0
        self.reference = None
        self.command_error = None
        self.previous_reason = None
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
        self.last_status = 0.0
        self.frames = 0
        self.successes = 0
        self.consecutive = 0
        self.origin_xy = None
        self.latest = None
        self.saved_first = False
        self.title = 'H Robot - Vision Only (R: reacquire, C: clear trail, Q: close)'
        if self.show_window:
            cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.title, 1100, 920)
            cv2.setMouseCallback(self.title, self.on_click)
        metadata = {'calibration_file': self.mapper.filename, 'plane_world_z_m': self.mapper.z,
                    'camera_crop': 'full sensor area only', 'pose_is_plane_projection': True,
                    'orientation_estimated': False, 'world_mapping_physically_validated_this_session': False,
                    'controls_robot': False, 'controls_motor': False,
                    'timestamp_source': 'host receipt, not hardware trigger',
                    'tracking_score_is_probability': False,
                    'video': 'Bounded unannotated MJPG clips with per-frame center coordinates',
                    'recording_filter': {
                        'signal': 'world XY silhouette center only',
                        'movement_threshold_mm': self.recorder.movement_threshold_mm,
                        'minimum_detection_fraction': self.recorder.minimum_detection_fraction,
                        'static_clips_are_deleted': True,
                        'insufficient_tracking_is_kept': True,
                        'kept_clip_trajectory': 'trajectory.png'},
                    'events': 'Operator time markers only; no motor feedback or control',
                    'reference_paths': 'Visual only; no robot reachability or collision validation'}
        (self.session/'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        self.get_logger().info(f'Vision only, recording to {self.session}')

    def on_click(self, event, x, y, flags, parameter):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.reset((x, y))

    def reset(self, seed=None):
        self.recorder.mark_tracking_discontinuity()
        self.detector.reset(seed)
        self.trail.clear(); self.samples.clear()
        self.origin_xy = None; self.consecutive = 0
        self.reference = None
        self.event('target_reset', seed_half_px=seed)

    def event(self, label, **fields):
        entry = {'utc': datetime.now(timezone.utc).isoformat(), 'wall_time_ns': time.time_ns(),
                 'event': label, **fields}
        self.events.write(json.dumps(entry)+'\n')
        self.events.flush()

    def handle_command(self, command, value):
        self.command_error = None
        if command in ('reset', 'select'):
            self.reset(value)
        elif command == 'clear':
            self.trail.clear(); self.samples.clear(); self.origin_xy = None
            self.event('trail_cleared')
        elif command == 'record_start':
            if self.last_frame is None or time.monotonic()-self.last_frame > .25:
                raise RuntimeError('相机无新图像，未开始录像')
            self.recorder.start(value)
            if self.web:
                self.web.trajectory = None
            self.event('recording_started', directory=str(self.recorder.directory), duration_s=value,
                       motor_command_sent=False)
        elif command == 'record_stop':
            self.recorder.stop()
            self.event('recording_stopped', motor_command_sent=False)
        elif command == 'event':
            self.event(value, source='operator_click_not_hardware_feedback', motor_command_sent=False)
        elif command == 'path':
            kind, size = value
            if kind == 'clear':
                self.reference = None
            else:
                if not self.latest or not self.latest.get('detected') or self.last_frame is None or time.monotonic()-self.last_frame > .2:
                    raise RuntimeError('请先锁定静止的 H，再设置参考路径')
                if self.consecutive < 20 or len(self.samples) < 20 or np.max(np.std(np.asarray(self.samples)[-20:], axis=0)) > .2:
                    raise RuntimeError('H 尚未稳定，请静止约 1 秒再设置参考路径')
                start = np.mean(np.asarray(self.samples)[-20:], axis=0)/1000
                points = reference_path(kind, start, size)
                pixels = self.mapper.image_points(points, (1224, 1024))
                if np.any(pixels < [86, 280]) or np.any(pixels > [1138, 952]):
                    raise RuntimeError('参考路径超出当前可用画面；这不是机械臂可达性判断')
                self.reference = {'kind': kind, 'size_mm': size, 'points_world_m': points.tolist(),
                                  'controls_robot': False, 'reachability_checked': False}
            self.event('reference_path_changed', reference=self.reference)

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
        self.event('recording_finalized', clip_name=status.get('clip_name'),
                   directory=status.get('directory'), kept=status.get('kept'),
                   classification=status.get('classification'),
                   reason=status.get('reason'), motion=status.get('motion'),
                   trajectory_path=status.get('trajectory_path'),
                   motor_command_sent=False)

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
                    self.event('command_rejected', command=command, reason=str(error))
        self.recorder.tick()
        self.sync_recording_result()
        if self.latest is not None:
            self.latest.update(recording=self.recorder.status(), command_error=self.command_error,
                               reference_path=self.reference)
        if self.last_frame is None or time.monotonic()-self.last_frame > .25:
            self.publish_invalid()
            if self.detector.initialized:
                self.detector.locked_out = True
            if self.latest is not None:
                self.latest['detected'] = False
                self.latest['stream_stale'] = True
                self.latest['tracking_reason'] = 'stream_timeout'
                self.write_status()
        if self.show_window:
            key = cv2.waitKey(1) & 0xff
            if key == ord('r'):
                self.reset()
            elif key == ord('c'):
                self.trail.clear();self.samples.clear();self.origin_xy = None
            elif key == ord('q') or cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) < 1:
                rclpy.shutdown()

    def write_status(self):
        # Atomic replace of our own status artifact for observers of this session.
        temporary = self.session/'status.tmp'
        temporary.write_text(json.dumps(self.latest, ensure_ascii=False, indent=2), encoding='utf-8')
        temporary.replace(self.session/'status.json')
        if self.web:self.web.status = json.dumps(self.latest).encode()
        self.log.flush()
        self.diagnostics_log.flush()

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
            if self.detector.initialized:
                self.detector.locked_out = True
            diagnostic = {'reason': 'image_timestamp_stale'}
        preview = image.copy()
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        reason = diagnostic['reason']
        xy = full_pixel = None
        if result is not None:
            xy, full_pixel = self.mapper.project(result['pixel'], (1224, 1024))
        self.recorder.write(image, stamp, result is not None, reason,
                            None if xy is None else xy*1000)
        self.sync_recording_result()
        self.diagnostics_log.write(json.dumps({'host_frame_time_ns': stamp,
            'frame': self.frames, 'image_age_s': age, **diagnostic})+'\n')
        if reason != self.previous_reason:
            self.event('tracking_state_changed', reason=reason, host_frame_time_ns=stamp)
            self.previous_reason = reason
        self.latest = {'utc': datetime.now(timezone.utc).isoformat(), 'host_frame_time_ns': stamp,
                       'detected': result is not None, 'frames': self.frames,
                       'stream_stale': not fresh, 'world_mapping_verified': False,
                       'session_directory': str(self.session), 'tracking_reason': reason,
                       'tracking_diagnostics': diagnostic, 'recording': self.recorder.status(),
                       'command_error': self.command_error, 'reference_path': self.reference}
        if self.reference is not None:
            vertices = np.asarray(self.reference['points_world_m'])
            dense = np.concatenate([np.linspace(a, b, 20) for a, b in zip(vertices[:-1], vertices[1:])])
            pixels = np.rint(self.mapper.image_points(dense, (1224, 1024))).astype(np.int32)
            cv2.polylines(preview, [pixels], False, (255, 130, 40), 2)
            for i, pixel in enumerate(self.mapper.image_points(vertices[:-1] if self.reference['kind']=='square' else vertices, (1224, 1024))):
                point = tuple(np.rint(pixel).astype(int))
                cv2.circle(preview, point, 5, (255, 130, 40), -1)
                cv2.putText(preview, str(i), (point[0]+8, point[1]-8), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 130, 40), 2)
        lines = ['VISION ONLY - no robot/motor commands',
                 'World XY: existing calibration; verify with a known physical distance']
        if result is None:
            self.publish_invalid()
            self.writer.writerow([stamp, 0, '', '', '', '', '', 0])
            lines += [f'TARGET LOST ({reason}) - pose paused; click H to reacquire']
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
            self.writer.writerow([stamp, 1, *full_pixel, *(xy*1000), result['area_px'], result['confidence']])
            spread = np.std(np.asarray(self.samples), axis=0)
            delta = (xy-self.origin_xy)*1000
            self.latest.update({'pixel_full': full_pixel.tolist(), 'world_xy_mm': (xy*1000).tolist(),
                                'window_std_mm': spread.tolist(), 'window_samples': len(self.samples),
                                'consecutive_detections': self.consecutive, 'area_half_px': result['area_px'],
                                'displacement_from_reset_mm': delta.tolist()})
            lines += [f'TRACKED H silhouette: X={xy[0]*1000:.2f} mm  Y={xy[1]*1000:.2f} mm',
                      f'Full pixels: ({full_pixel[0]:.2f}, {full_pixel[1]:.2f})',
                      f'Displacement: dX={delta[0]:+.2f} mm  dY={delta[1]:+.2f} mm',
                      f'Window std (stationary only): X={spread[0]:.3f}  Y={spread[1]:.3f} mm',
                      'Green box = selected target; red cross = silhouette centroid']
            if not self.saved_first:
                cv2.imwrite(str(self.session/'first_frame.png'), image)
                self.saved_first = True
        fps = (len(self.arrival_times)-1)/(self.arrival_times[-1]-self.arrival_times[0]) if len(self.arrival_times)>1 else 0
        self.latest['fps'] = fps
        self.latest['detection_fraction'] = self.successes/self.frames
        lines += [f'{fps:.1f} FPS | detection {self.successes}/{self.frames} | R=reacquire  C=clear trail  Q=close']
        record = self.recorder.status()
        record_text = f'RECORDING {record["frames"]} frames, {record["seconds_remaining"]:.0f}s left' if record['active'] else 'VIDEO OFF'
        lines += [record_text+' | recording NEVER stops motor']
        cv2.rectangle(preview, (0, 0), (1224, len(lines)*28+12), (32, 32, 32), -1)
        for i, line in enumerate(lines):
            cv2.putText(preview, line, (12, 27+i*28), cv2.FONT_HERSHEY_SIMPLEX, .59, (230, 230, 230), 1, cv2.LINE_AA)
        msg = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8');msg.header = message.header
        self.preview_pub.publish(msg)
        if self.show_window:
            cv2.imshow(self.title, preview)
        if self.web:
            self.web.status = json.dumps(self.latest).encode()
            if self.frames % 4 == 0:
                self.web.jpeg = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
        if now-self.last_status >= 1:
            self.write_status();self.last_status = now
            cv2.imwrite(str(self.session/'latest_preview.jpg'), preview)

    def destroy_node(self):
        self.recorder.stop('viewer_closed')
        self.sync_recording_result()
        if self.latest is not None:
            self.latest['detected'] = False
            self.latest['viewer_closed'] = True
            self.latest['recording'] = self.recorder.status()
            self.write_status()
        self.log.flush();self.log.close()
        self.events.close(); self.diagnostics_log.close()
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
