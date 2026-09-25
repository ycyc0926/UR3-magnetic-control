from pathlib import Path
import copy
import tempfile
import unittest

import yaml

from ur3_magnetic_control.acrylic_ceiling_guard import (
    AcrylicCeilingGuard,
    CEILING_OBJECT_ID,
    LEFT_OBJECT_ID,
    RIGHT_OBJECT_ID,
    load_guard_configuration,
    transform_point,
)
from shape_msgs.msg import SolidPrimitive
from ur3_magnetic_control.acrylic_geometry import (
    acrylic_ceiling_gap_m,
    acrylic_inner_lower_left_world_xy_m,
    modeled_boundary_gaps_m,
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
        self.assertEqual(
            configuration["acrylic_inner_lower_left_world_xy_m"], [0.135, 0.025]
        )
        self.assertAlmostEqual(configuration["ceiling_region_min_world_y_m"], 0.025)

    def test_ceiling_exemption_requires_complete_geometry_below_lower_edge(self):
        table = yaml.safe_load(
            (DEFAULT_PROJECT_ROOT / "config/table_world_calibration.yaml").read_text()
        )
        self.assertEqual(acrylic_inner_lower_left_world_xy_m(table), (0.135, 0.025))
        self.assertEqual(
            acrylic_ceiling_gap_m(table, [0.0, 0.010, 0.0], [0.1, 0.0249, 0.60]),
            float("inf"),
        )
        for lower_y, upper_y in [(0.010, 0.025), (0.010, 0.030), (0.025, 0.030)]:
            with self.subTest(lower_y=lower_y, upper_y=upper_y):
                self.assertAlmostEqual(
                    acrylic_ceiling_gap_m(
                        table, [0.0, lower_y, 0.0], [0.1, upper_y, 0.480]
                    ),
                    0.010741,
                )

    def test_x_corner_is_recorded_but_does_not_create_an_x_exemption(self):
        table = yaml.safe_load(
            (DEFAULT_PROJECT_ROOT / "config/table_world_calibration.yaml").read_text()
        )
        sides = yaml.safe_load(
            (DEFAULT_PROJECT_ROOT / "config/acrylic_side_boundaries.yaml").read_text()
        )
        gaps = modeled_boundary_gaps_m(
            table, sides, [-0.20, 0.030, 0.0], [0.10, 0.040, 0.480]
        )
        self.assertAlmostEqual(gaps["ceiling"], 0.010741)

    def test_side_panels_do_not_cover_space_before_y_25_mm(self):
        table = yaml.safe_load(
            (DEFAULT_PROJECT_ROOT / "config/table_world_calibration.yaml").read_text()
        )
        sides = yaml.safe_load(
            (DEFAULT_PROJECT_ROOT / "config/acrylic_side_boundaries.yaml").read_text()
        )
        before = modeled_boundary_gaps_m(
            table, sides, [-0.20, -0.12, 0.0], [-0.18, -0.109, 0.4]
        )
        self.assertEqual(before["left"], float("inf"))
        self.assertEqual(before["right"], float("inf"))
        touching = modeled_boundary_gaps_m(
            table, sides, [-0.20, 0.020, 0.0], [-0.18, 0.025, 0.4]
        )
        self.assertAlmostEqual(touching["left"], -0.060)

    def test_missing_or_ambiguous_footprint_fails_closed(self):
        table = yaml.safe_load(
            (DEFAULT_PROJECT_ROOT / "config/table_world_calibration.yaml").read_text()
        )
        variants = []
        missing = copy.deepcopy(table)
        del missing["fixed_work_surface"]["acrylic_footprint"]
        variants.append(missing)
        wrong_model = copy.deepcopy(table)
        wrong_model["fixed_work_surface"]["acrylic_footprint"][
            "ceiling_region_model"
        ] = "unknown"
        variants.append(wrong_model)
        nonfinite = copy.deepcopy(table)
        nonfinite["fixed_work_surface"]["acrylic_footprint"][
            "inner_lower_left_world_xy_m"
        ] = [0.135, float("nan")]
        variants.append(nonfinite)
        for variant in variants:
            with self.subTest(variant=variant), self.assertRaises(ValueError):
                acrylic_ceiling_gap_m(
                    variant, [0.0, 0.0, 0.0], [0.1, 0.020, 0.480]
                )

    def test_moveit_ceiling_box_starts_at_measured_lower_y_edge(self):
        guard = object.__new__(AcrylicCeilingGuard)
        guard.configuration = load_guard_configuration()
        item = guard._forbidden_ceiling_object()
        self.assertEqual(item.id, CEILING_OBJECT_ID)
        self.assertEqual(list(item.primitives[0].dimensions), [2.0, 2.0, 1.0])
        expected = transform_point(
            guard.configuration["T_base_from_world"],
            [0.0, 1.025, guard.configuration["guard_world_z_m"] + 0.5],
        )
        actual = item.primitive_poses[0].position
        self.assertAlmostEqual(actual.x, expected[0])
        self.assertAlmostEqual(actual.y, expected[1])
        self.assertAlmostEqual(actual.z, expected[2])

    def test_moveit_scene_contains_both_side_forbidden_regions(self):
        guard = object.__new__(AcrylicCeilingGuard)
        guard.configuration = load_guard_configuration()
        objects = guard._forbidden_side_objects()
        self.assertEqual([item.id for item in objects], [LEFT_OBJECT_ID, RIGHT_OBJECT_ID])
        self.assertTrue(
            all(list(item.primitives[0].dimensions) == [1.0, 2.0, 2.0] for item in objects)
        )
        expected_y = transform_point(
            guard.configuration["T_base_from_world"],
            [guard.configuration["left_guard_world_x_m"] - 0.5, 1.025, 0.5],
        )
        actual = objects[0].primitive_poses[0].position
        self.assertAlmostEqual(actual.x, expected_y[0])
        self.assertAlmostEqual(actual.y, expected_y[1])

    def test_attached_magnet_encloses_all_motor_phases(self):
        guard = object.__new__(AcrylicCeilingGuard)
        guard.configuration = load_guard_configuration()
        collision = guard._attached_magnet_cylinder().object
        self.assertEqual(collision.primitives[0].type, SolidPrimitive.CYLINDER)
        self.assertAlmostEqual(collision.primitives[0].dimensions[0], 0.030)
        self.assertAlmostEqual(collision.primitives[0].dimensions[1],
                               (0.015 ** 2 + 0.015 ** 2) ** 0.5)
        pose = collision.primitive_poses[0]
        self.assertAlmostEqual(pose.position.z, 0.0975)
        self.assertAlmostEqual(pose.orientation.y, 0.0)
        self.assertAlmostEqual(pose.orientation.w, 1.0)

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
