"""Pure checks for one manual-motor probe. No robot or motor command APIs."""
import json
from pathlib import Path

import numpy as np

MEASURED_TCP_SPEED_LIMIT_MM_S = 20.0
MEASURED_JOINT_SPEED_LIMIT_DEG_S = 5.0


def check_tcp_speed(state):
    speed = np.asarray(state['actual_TCP_speed'], float)
    if speed.shape != (6,) or not np.isfinite(speed).all():
        raise RuntimeError('Invalid direct TCP speed')
    if np.linalg.norm(speed[:3])*1000 > MEASURED_TCP_SPEED_LIMIT_MM_S:
        raise RuntimeError(f'Measured TCP speed exceeded {MEASURED_TCP_SPEED_LIMIT_MM_S:g} mm/s; stop motor physically')


def check_probe_vision(value, start_xy_mm):
    """Caller must first validate timestamp, stream, identity and reader freshness."""
    point = np.asarray(value['world_xy_mm'], float)
    pixel = (np.asarray(value['pixel_full'], float)+.5)/2-.5
    if point.shape != (2,) or pixel.shape != (2,) or not np.isfinite([point, pixel]).all():
        raise RuntimeError('Invalid probe observation')
    dx, dy = point-np.asarray(start_xy_mm, float)
    if not -15 <= dx <= 1 or abs(dy) > 3.5:
        raise RuntimeError('H left the reviewed -X observation corridor; stop motor physically')
    if np.any(pixel < [136, 122]) or np.any(pixel > [1088, 902]):
        raise RuntimeError('H approached image boundary; stop motor physically')
    return point


class MotorMarkerGate:
    """Human UI markers coordinate a single attempt; they do not sense the motor."""
    def __init__(self, path, armed_ns, max_duration_s=6.):
        self.path = Path(path)
        self.armed_ns = int(armed_ns)
        self.start_event = None
        self.max_duration_s = float(max_duration_s)

    def events(self):
        events = []
        for line in self.path.read_text().splitlines():
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                # A concurrent writer can leave a partial final line temporarily.
                continue
            if (event.get('wall_time_ns', 0) >= self.armed_ns and
                    event.get('event') in ('motor_started', 'motor_stopped', 'target_reset')):
                events.append(event)
        return events

    def ready(self, now_ns):
        events = self.events()
        if any(e['event'] in ('motor_stopped', 'target_reset') for e in events):
            raise RuntimeError('Stop or target reset after arming; no automatic restart')
        starts = [e for e in events if e['event'] == 'motor_started']
        if len(starts) > 1:
            raise RuntimeError('Multiple motor start markers; no automatic retry')
        if not starts:
            return False
        age = (now_ns-starts[0]['wall_time_ns'])/1e9
        if not 0 <= age <= .5:
            raise RuntimeError('Motor start marker stale/future; no trajectory')
        if starts[0].get('source') != 'operator_click_not_hardware_feedback':
            raise RuntimeError('Unknown start marker source')
        self.start_event = starts[0]
        return True

    def check_during(self, now_ns):
        if self.start_event is None:
            raise RuntimeError('No accepted motor start marker')
        events = self.events()
        if any(e['event'] in ('motor_stopped', 'target_reset') for e in events):
            raise RuntimeError('Operator stopped motor or reselected H; stop arm')
        if len([e for e in events if e['event'] == 'motor_started']) != 1:
            raise RuntimeError('Unexpected repeated start; stop arm')
        if not 0 <= (now_ns-self.start_event['wall_time_ns'])/1e9 <= self.max_duration_s:
            raise RuntimeError('Manual motor trial window ended; stop arm and motor')
