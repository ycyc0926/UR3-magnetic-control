import struct
import unittest
from rtde_status_outputs import FIELDS, FORMATS, parse


class StatusPacketTests(unittest.TestCase):
    def packet(self, overrides=None):
        overrides = overrides or {}
        values = {'timestamp': 100., 'actual_q': [0.]*6, 'actual_qd': [0.]*6,
                  'actual_TCP_pose': [.1, -.3, .4, 0, 1.5, 0], 'actual_TCP_speed': [0.]*6,
                  'tcp_offset': [0, -.062, .041, 0, 0, 0], 'payload': .32,
                  'payload_cog': [0, -.062, .041], 'speed_scaling': 0., 'target_speed_fraction': 1.,
                  'runtime_state': 1, 'robot_mode': 7, 'safety_mode': 1}
        values.update(overrides)
        return b'\x02'+b''.join(struct.pack('>'+FORMATS[kind], *(values[name] if isinstance(values[name], list) else [values[name]]))
                                for name, kind in FIELDS)

    def test_status_fields_and_tcp_decode(self):
        result = parse(self.packet(), 2)
        self.assertEqual(result['runtime_state'], 1)
        self.assertEqual(result['safety_mode'], 1)
        self.assertEqual(result['tcp_offset'], (0, -.062, .041, 0, 0, 0))

    def test_invalid_size_or_recipe_is_rejected(self):
        for data, recipe in [(self.packet()[:-1], 2), (self.packet(), 3), (b'', 2)]:
            with self.assertRaises(RuntimeError): parse(data, recipe)

    def test_nonfinite_input_is_rejected(self):
        for key, value in [('payload', float('nan')), ('actual_qd', [float('inf')]*6)]:
            with self.assertRaises(RuntimeError): parse(self.packet({key: value}), 2)


if __name__ == '__main__': unittest.main()
