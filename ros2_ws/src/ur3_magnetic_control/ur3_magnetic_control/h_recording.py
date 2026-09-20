"""Bounded, unannotated video and per-frame timestamps; NEVER controls a motor."""
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import time

import cv2
import numpy as np


class BoundedRecorder:
    def __init__(self, session, fps=20.0, reserve_bytes=2*1024**3,
                 max_bytes=512*1024**2):
        self.session = Path(session)
        self.fps = fps
        self.reserve_bytes = reserve_bytes
        self.max_bytes = max_bytes
        self.active = False
        self.video = None
        self.log = None
        self.directory = None
        self.frames = 0
        self.last_stamp = None
        self.max_gap_s = 0.0
        self.reason = 'not_started'
        self.deadline = 0.0
        self.size = None
        self.last_error = None

    def start(self, duration_s=120.0):
        if not np.isfinite(duration_s) or not 5 <= duration_s <= 180:
            raise ValueError('Recording duration must be 5--180 s')
        if self.active:
            raise RuntimeError('A recording is already active')
        if shutil.disk_usage(self.session).free < self.reserve_bytes:
            raise RuntimeError('Less than reserved free disk space')
        self.directory = self.session/('clip_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.directory.mkdir(exist_ok=False)
        self.log = (self.directory/'frame_times.csv').open('x', newline='', encoding='utf-8')
        self.csv = csv.writer(self.log)
        self.csv.writerow(['video_frame_index', 'host_frame_time_ns', 'receipt_wall_time_ns',
                           'detected', 'tracking_reason'])
        self.frames = 0
        self.last_stamp = None
        self.max_gap_s = 0.0
        self.size = None
        self.last_error = None
        self.reason = 'recording'
        self.deadline = time.monotonic()+duration_s
        self.active = True
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self.duration_s = duration_s
        self._save_metadata()

    def _save_metadata(self):
        if self.directory is None:
            return
        data = self.status()
        data.update(started_utc=self.started_utc, requested_duration_s=self.duration_s,
                    codec='MJPG', lossy=True, annotated=False,
                    nominal_playback_fps=self.fps, image_size_px=self.size,
                    timestamp_source='host frame receipt, not sensor exposure',
                    timing_authority='frame_times.csv; do not infer timing solely from AVI fps',
                    controls_motor=False, controls_robot=False,
                    recording_timeout_does_not_stop_motor=True)
        temporary = self.directory/'metadata.tmp'
        temporary.write_text(json.dumps(data, indent=2), encoding='utf-8')
        temporary.replace(self.directory/'metadata.json')

    def status(self):
        return {'active': self.active, 'frames': self.frames, 'reason': self.reason,
                'seconds_remaining': max(0.0, self.deadline-time.monotonic()) if self.active else 0.0,
                'directory': str(self.directory) if self.directory else None,
                'max_received_frame_gap_s': self.max_gap_s, 'error': self.last_error,
                'motor_control': False}

    def tick(self):
        if self.active and time.monotonic() >= self.deadline:
            self.stop('recording_time_limit')

    def write(self, image, stamp_ns, detected, reason):
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
            self.video.write(image)
            self.csv.writerow([self.frames, stamp_ns, time.time_ns(), int(detected), reason])
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

    def stop(self, reason='operator_stopped_recording'):
        if not self.active:
            return
        self.active = False
        self.reason = reason
        if self.video is not None:
            self.video.release()
            self.video = None
        if self.log is not None:
            self.log.close()
            self.log = None
        self._save_metadata()
