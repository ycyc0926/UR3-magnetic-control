"""Bounded video, center trajectory, and conservative static-clip filtering."""
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import time

import cv2
import numpy as np


def _rolling_median(points, window=5):
    """Suppress isolated center glitches without hiding sustained movement."""
    points = np.asarray(points, dtype=float)
    if len(points) < window:
        return np.median(points, axis=0, keepdims=True) if len(points) else points.reshape(0, 2)
    return np.asarray([np.median(points[index:index+window], axis=0)
                       for index in range(len(points)-window+1)])


def _directional_diameter(points):
    """Approximate the 2-D diameter to within the 2.5 degree direction grid."""
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return 0.0
    angles = np.deg2rad(np.arange(0.0, 180.0, 5.0))
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    projections = points @ directions.T
    return float(np.max(np.ptp(projections, axis=0)))


class BoundedRecorder:
    def __init__(self, session, fps=20.0, reserve_bytes=2*1024**3,
                 max_bytes=512*1024**2, movement_threshold_mm=0.5,
                 minimum_detection_fraction=0.8):
        self.session = Path(session)
        self.fps = fps
        self.reserve_bytes = reserve_bytes
        self.max_bytes = max_bytes
        self.movement_threshold_mm = float(movement_threshold_mm)
        self.minimum_detection_fraction = float(minimum_detection_fraction)
        if not np.isfinite(self.movement_threshold_mm) or self.movement_threshold_mm <= 0:
            raise ValueError('Movement threshold must be positive')
        if not 0 < self.minimum_detection_fraction <= 1:
            raise ValueError('Detection fraction must be in (0, 1]')
        self.active = False
        self.video = None
        self.log = None
        self.directory = None
        self.frames = 0
        self.last_stamp = None
        self.max_gap_s = 0.0
        self.reason = 'not_started'
        self.stop_reason = None
        self.deadline = 0.0
        self.size = None
        self.last_error = None
        self.classification = None
        self.kept = None
        self.motion = None
        self.trajectory_path = None
        self.clip_name = None
        self.discarded_clip = None
        self.finalization_serial = 0
        self.positions = []
        self.discontinuities = []

    def start(self, duration_s=120.0, session=None):
        if not np.isfinite(duration_s) or not 5 <= duration_s <= 180:
            raise ValueError('Recording duration must be 5--180 s')
        if self.active:
            raise RuntimeError('A recording is already active')
        destination = self.session if session is None else Path(session)
        storage = destination
        while not storage.exists():
            storage = storage.parent
        if shutil.disk_usage(storage).free < self.reserve_bytes:
            raise RuntimeError('Less than reserved free disk space')
        self.session = destination
        self.clip_name = 'clip_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory = self.session/self.clip_name
        self.directory.mkdir(parents=True, exist_ok=False)
        self.log = (self.directory/'frame_times.csv').open('x', newline='', encoding='utf-8')
        self.csv = csv.writer(self.log)
        self.csv.writerow(['video_frame_index', 'host_frame_time_ns', 'receipt_wall_time_ns',
                           'detected', 'tracking_reason', 'world_x_mm', 'world_y_mm'])
        self.frames = 0
        self.last_stamp = None
        self.max_gap_s = 0.0
        self.size = None
        self.last_error = None
        self.classification = None
        self.kept = None
        self.motion = None
        self.trajectory_path = None
        self.discarded_clip = None
        self.positions = []
        self.discontinuities = []
        self.reason = 'recording'
        self.stop_reason = None
        self.deadline = time.monotonic()+duration_s
        self.active = True
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self.duration_s = duration_s
        self._save_metadata()

    def mark_tracking_discontinuity(self):
        """Prevent manual/automatic target reacquisition from looking like motion."""
        if self.active and (not self.discontinuities or self.discontinuities[-1] != self.frames):
            self.discontinuities.append(self.frames)

    def _save_metadata(self):
        if self.directory is None or not self.directory.exists():
            return
        data = self.status()
        data.update(started_utc=self.started_utc, requested_duration_s=self.duration_s,
                    codec='MJPG', lossy=True, annotated=False,
                    nominal_playback_fps=self.fps, image_size_px=self.size,
                    timestamp_source='host frame receipt, not sensor exposure',
                    timing_authority='frame_times.csv; do not infer timing solely from AVI fps',
                    controls_motor=False, controls_robot=False,
                    recording_timeout_does_not_stop_motor=True,
                    static_filter={'signal': 'calibrated world XY center only',
                                   'movement_threshold_mm': self.movement_threshold_mm,
                                   'minimum_detection_fraction': self.minimum_detection_fraction,
                                   'rolling_median_frames': 5,
                                   'maximum_bridge_gap_frames': 4,
                                   'insufficient_tracking_is_kept': True})
        temporary = self.directory/'metadata.tmp'
        temporary.write_text(json.dumps(data, indent=2), encoding='utf-8')
        temporary.replace(self.directory/'metadata.json')

    def status(self):
        trajectory_available = (self.trajectory_path is not None and
                                self.trajectory_path.exists())
        return {'active': self.active, 'frames': self.frames, 'reason': self.reason,
                'stop_reason': self.stop_reason,
                'seconds_remaining': max(0.0, self.deadline-time.monotonic()) if self.active else 0.0,
                'directory': str(self.directory) if self.directory else None,
                'clip_name': self.clip_name, 'discarded_clip': self.discarded_clip,
                'max_received_frame_gap_s': self.max_gap_s, 'error': self.last_error,
                'classification': self.classification, 'kept': self.kept,
                'motion': self.motion, 'trajectory_available': trajectory_available,
                'trajectory_path': str(self.trajectory_path) if trajectory_available else None,
                'finalization_serial': self.finalization_serial,
                'motor_control': False}

    def tick(self):
        if self.active and time.monotonic() >= self.deadline:
            self.stop('recording_time_limit')

    def write(self, image, stamp_ns, detected, reason, world_xy_mm=None):
        self.tick()
        if not self.active:
            return
        try:
            size = (image.shape[1], image.shape[0])
            if self.video is None:
                self.size = size
                self.video = cv2.VideoWriter(str(self.directory/'unannotated.avi'),
                    cv2.CAP_FFMPEG, cv2.VideoWriter_fourcc(*'MJPG'), self.fps, size)
                if not self.video.isOpened():
                    raise RuntimeError('MJPG video writer did not open')
            if size != self.size:
                raise RuntimeError('Image size changed during recording')
            if self.frames % 20 == 0:
                if shutil.disk_usage(self.session).free < self.reserve_bytes:
                    raise RuntimeError('Free disk space fell below reserve')
                if (self.directory/'unannotated.avi').stat().st_size >= self.max_bytes:
                    self.stop('recording_size_limit')
                    return
            position = None
            if detected and world_xy_mm is not None:
                candidate = np.asarray(world_xy_mm, dtype=float)
                if candidate.shape == (2,) and np.isfinite(candidate).all():
                    position = candidate
            self.video.write(image)
            self.csv.writerow([self.frames, stamp_ns, time.time_ns(), int(detected), reason,
                               '' if position is None else float(position[0]),
                               '' if position is None else float(position[1])])
            self.positions.append(position)
            if self.last_stamp is not None:
                self.max_gap_s = max(self.max_gap_s, (stamp_ns-self.last_stamp)/1e9)
            self.last_stamp = stamp_ns
            self.frames += 1
            if self.frames % 20 == 0:
                self.log.flush()
                self._save_metadata()
        except (OSError, RuntimeError, cv2.error) as error:
            self.last_error = str(error)
            self.stop('recording_error')

    def _segments(self, bridge_gap):
        segments = []
        current = []
        missing = 0
        breaks = set(self.discontinuities)
        for index, position in enumerate(self.positions):
            if index in breaks and current:
                segments.append(np.asarray(current))
                current = []
                missing = 0
            if position is None:
                missing += 1
                continue
            if current and missing > bridge_gap:
                segments.append(np.asarray(current))
                current = []
            current.append(position)
            missing = 0
        if current:
            segments.append(np.asarray(current))
        return segments

    def _maximum_missing_run(self):
        maximum = current = 0
        for position in self.positions:
            if position is None:
                current += 1
                maximum = max(maximum, current)
            else:
                current = 0
        return maximum

    def _assess_motion(self):
        valid = [position for position in self.positions if position is not None]
        detection_fraction = len(valid)/self.frames if self.frames else 0.0
        segments = self._segments(bridge_gap=4)
        filtered = [_rolling_median(segment) for segment in segments if len(segment)]
        spans = [_directional_diameter(segment) for segment in filtered]
        center_span = max(spans, default=0.0)
        path_length = 0.0
        for segment in filtered:
            if len(segment) > 1:
                path_length += float(np.sum(np.linalg.norm(np.diff(segment, axis=0), axis=1)))
        if valid:
            edge = max(1, min(len(valid)//4, round(self.fps)))
            start = np.median(np.asarray(valid[:edge]), axis=0)
            end = np.median(np.asarray(valid[-edge:]), axis=0)
            net = float(np.linalg.norm(end-start))
            start_xy, end_xy = start.tolist(), end.tolist()
        else:
            net = None
            start_xy = end_xy = None
        max_missing = self._maximum_missing_run()
        enough_motion_evidence = any(len(segment) >= 5 for segment in segments)
        moving = enough_motion_evidence and center_span >= self.movement_threshold_mm
        reliable_static = (self.frames >= 20 and len(valid) >= 20 and
                           detection_fraction >= self.minimum_detection_fraction and
                           max_missing < 5 and not self.discontinuities)
        if moving:
            classification = 'moving'
            decision = 'kept_center_moved'
        elif reliable_static:
            classification = 'static'
            decision = 'discarded_no_center_motion'
        else:
            classification = 'indeterminate'
            decision = 'kept_insufficient_tracking_for_safe_discard'
        summary = {'classification': classification, 'decision': decision,
                   'center_span_mm': center_span, 'movement_threshold_mm': self.movement_threshold_mm,
                   'filtered_path_length_mm': path_length, 'net_displacement_mm': net,
                   'start_world_xy_mm': start_xy, 'end_world_xy_mm': end_xy,
                   'position_frames': len(valid), 'total_frames': self.frames,
                   'detection_fraction': detection_fraction,
                   'minimum_detection_fraction': self.minimum_detection_fraction,
                   'max_consecutive_missing_frames': max_missing,
                   'target_reselections_during_recording': len(self.discontinuities)}
        return classification, summary

    def _draw_trajectory(self):
        segments = [_rolling_median(segment) for segment in self._segments(bridge_gap=0)
                    if len(segment)]
        if not segments:
            return None
        points = np.concatenate([segment for segment in segments if len(segment)], axis=0)
        if not len(points):
            return None
        canvas = np.full((800, 1200, 3), 248, np.uint8)
        left, top, right, bottom = 90, 70, 890, 730
        center = np.mean([np.min(points, axis=0), np.max(points, axis=0)], axis=0)
        span = max(float(np.ptp(points[:, 0])), float(np.ptp(points[:, 1])), 1.0)*1.2
        x0, x1 = center[0]-span/2, center[0]+span/2
        y0, y1 = center[1]-span/2, center[1]+span/2

        def pixel(point):
            x = left+(point[0]-x0)/(x1-x0)*(right-left)
            y = bottom-(point[1]-y0)/(y1-y0)*(bottom-top)
            return tuple(np.rint([x, y]).astype(int))

        cv2.putText(canvas, 'H center trajectory (calibrated world XY)', (40, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, .85, (25, 25, 25), 2, cv2.LINE_AA)
        for index in range(6):
            fraction = index/5
            x = round(left+fraction*(right-left)); y = round(top+fraction*(bottom-top))
            cv2.line(canvas, (x, top), (x, bottom), (215, 215, 215), 1)
            cv2.line(canvas, (left, y), (right, y), (215, 215, 215), 1)
            cv2.putText(canvas, f'{x0+fraction*(x1-x0):.2f}', (x-28, bottom+24),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (70, 70, 70), 1, cv2.LINE_AA)
            cv2.putText(canvas, f'{y1-fraction*(y1-y0):.2f}', (8, y+5),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (70, 70, 70), 1, cv2.LINE_AA)
        cv2.rectangle(canvas, (left, top), (right, bottom), (60, 60, 60), 2)
        cv2.putText(canvas, 'World X / mm', (420, 778), cv2.FONT_HERSHEY_SIMPLEX,
                    .58, (40, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(canvas, 'World Y / mm', (8, 62), cv2.FONT_HERSHEY_SIMPLEX,
                    .52, (40, 40, 40), 1, cv2.LINE_AA)
        total = max(1, sum(max(0, len(segment)-1) for segment in segments))
        progress = 0
        for segment in segments:
            for first, second in zip(segment, segment[1:]):
                fraction = progress/total
                color = (round(230*(1-fraction)), round(90+80*fraction), round(40+210*fraction))
                cv2.line(canvas, pixel(first), pixel(second), color, 2, cv2.LINE_AA)
                progress += 1
        cv2.circle(canvas, pixel(points[0]), 7, (30, 170, 30), -1, cv2.LINE_AA)
        cv2.circle(canvas, pixel(points[-1]), 7, (30, 30, 220), -1, cv2.LINE_AA)
        cv2.putText(canvas, 'START', (925, 85), cv2.FONT_HERSHEY_SIMPLEX, .55,
                    (30, 140, 30), 2, cv2.LINE_AA)
        cv2.putText(canvas, 'END', (925, 115), cv2.FONT_HERSHEY_SIMPLEX, .55,
                    (30, 30, 200), 2, cv2.LINE_AA)
        lines = [f'Class: {self.classification}',
                 f'Frames: {self.motion["position_frames"]}/{self.frames}',
                 f'Center span: {self.motion["center_span_mm"]:.3f} mm',
                 f'Threshold: {self.movement_threshold_mm:.3f} mm',
                 f'Net displacement: {self.motion["net_displacement_mm"]:.3f} mm'
                 if self.motion['net_displacement_mm'] is not None else 'Net displacement: n/a',
                 'Tracking gaps are not connected.']
        for index, line in enumerate(lines):
            cv2.putText(canvas, line, (925, 175+index*34), cv2.FONT_HERSHEY_SIMPLEX,
                        .48, (45, 45, 45), 1, cv2.LINE_AA)
        target = self.directory/'trajectory.png'
        if not cv2.imwrite(str(target), canvas):
            raise RuntimeError('Trajectory image could not be written')
        return target

    def stop(self, reason='operator_stopped_recording'):
        if not self.active:
            return
        self.active = False
        self.stop_reason = reason
        self.reason = reason
        if self.video is not None:
            self.video.release()
            self.video = None
        if self.log is not None:
            self.log.close()
            self.log = None
        if reason == 'recording_error':
            self.classification = 'error'
            self.kept = True
            self.motion = {'classification': 'error', 'decision': 'kept_recording_error'}
        else:
            self.classification, self.motion = self._assess_motion()
            self.kept = self.classification != 'static'
        if self.kept:
            try:
                if self.classification != 'error':
                    self.trajectory_path = self._draw_trajectory()
            except (OSError, RuntimeError, cv2.error, ValueError) as error:
                message = f'Trajectory generation failed: {error}'
                self.last_error = message if self.last_error is None else self.last_error+'; '+message
            self.finalization_serial += 1
            self._save_metadata()
            return

        self.reason = 'discarded_no_center_motion'
        self.finalization_serial += 1
        self._save_metadata()
        discarded_directory = self.directory
        try:
            shutil.rmtree(discarded_directory)
            self.discarded_clip = self.clip_name
            self.directory = None
            self.trajectory_path = None
        except OSError as error:
            self.classification = 'discard_failed'
            self.kept = True
            self.reason = 'static_discard_failed'
            self.last_error = f'Static clip could not be removed: {error}'
            self._save_metadata()
