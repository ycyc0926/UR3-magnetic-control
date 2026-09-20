from pathlib import Path
import tempfile
import unittest

import yaml

from ur3_magnetic_control.acrylic_ceiling_guard import (
    AcrylicCeilingGuard,
    LEFT_OBJECT_ID,
    RIGHT_OBJECT_ID,
    load_guard_configuration,
)
from ur3_magnetic_control.clearance_policy import (
    DEFAULT_PROJECT_ROOT,
    clearance_limits_mm,
    load_clearance_limits_m,
)


class ClearancePolicyTests(unittest.TestCase):
    def test_project_policy_is_the_only_active_limit_set(self):
        self.assertEqual(
            clearance_limits_mm(),
            {"ceiling": 5.0, "left": 10.0, "right": 10.0, "table": 10.0},
        )

    def test_one_side_value_controls_both_sides(self):
        limits = load_clearance_limits_m()
        self.assertEqual(limits["left"], limits["right"])

    def test_separate_left_or_right_override_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "config").mkdir()
            data = {
                "safety": {
                    "clearance_policy_m": {
                        "acrylic_bottom": 0.005,
                        "side": 0.010,
                        "left": 0.020,
                        "table": 0.010,
                    }
                }
            }
            (root / "config" / "ur3_system.yaml").write_text(
                yaml.safe_dump(data), encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "exactly"):
                load_clearance_limits_m(root)

    def test_moveit_guard_uses_the_same_policy(self):
        configuration = load_guard_configuration()
        self.assertEqual(configuration["clearance_limits_m"], load_clearance_limits_m())
        self.assertAlmostEqual(
            configuration["guard_world_z_m"],
            configuration["underside_world_z_m"] - 0.005,
        )
        self.assertAlmostEqual(configuration["left_guard_world_x_m"], -0.130)
        self.assertAlmostEqual(configuration["right_guard_world_x_m"], 0.580)

    def test_moveit_scene_contains_both_side_forbidden_regions(self):
        guard = object.__new__(AcrylicCeilingGuard)
        guard.configuration = load_guard_configuration()
        objects = guard._forbidden_side_objects()
        self.assertEqual([item.id for item in objects], [LEFT_OBJECT_ID, RIGHT_OBJECT_ID])
        self.assertTrue(
            all(list(item.primitives[0].dimensions) == [1.0, 2.0, 2.0] for item in objects)
        )

    def test_active_code_has_no_legacy_per_motion_clearance_settings(self):
        root = DEFAULT_PROJECT_ROOT
        files = [path for path in (root / "robot").glob("*.py") if not path.name.startswith("test_")]
        files += list((root / "config").glob("*.yaml"))
        files += list(
            (root / "ros2_ws/src/ur3_magnetic_control/ur3_magnetic_control").glob("*.py")
        )
        forbidden = (
            "GAP_LIMITS_M",
            "--nominal-ceiling-mm",
            "acrylic_ceiling_clearance_m",
            "diagnostic_clearance_m",
            "stopping_ceiling",
        )
        violations = {
            str(path): [token for token in forbidden if token in path.read_text(encoding="utf-8")]
            for path in files
        }
        self.assertFalse({path: tokens for path, tokens in violations.items() if tokens})


if __name__ == "__main__":
    unittest.main()
