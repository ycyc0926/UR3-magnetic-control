"""MoveIt collision guard for the finite acrylic underside and side planes.

The underside height guard begins at the measured world Y lower edge. Geometry
wholly below that edge is outside the acrylic footprint; geometry touching or
crossing it remains constrained. Side-panel guards remain global. All margins
come from the single project-wide clearance policy.
"""

from pathlib import Path
import copy
import math
import os

from geometry_msgs.msg import Pose
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive
import yaml

from .acrylic_geometry import acrylic_inner_lower_left_world_xy_m
from .clearance_policy import load_clearance_limits_m


DEFAULT_PROJECT_ROOT = Path("/home/yc/UR3")
CEILING_OBJECT_ID = "acrylic_ceiling_forbidden"
LEFT_OBJECT_ID = "acrylic_left_clearance_forbidden"
RIGHT_OBJECT_ID = "acrylic_right_clearance_forbidden"
MAGNET_OBJECT_ID = "magnet_cylinder_guard"
CEILING_REGION_SPAN_M = 2.0


def _project_root():
    return Path(os.environ.get("UR3_PROJECT_ROOT", DEFAULT_PROJECT_ROOT))


def load_guard_configuration():
    root = _project_root()
    with (root / "config" / "table_world_calibration.yaml").open(
        encoding="utf-8"
    ) as stream:
        world = yaml.safe_load(stream)
    with (root / "config" / "ur3_system.yaml").open(encoding="utf-8") as stream:
        system = yaml.safe_load(stream)
    with (root / "config" / "acrylic_side_boundaries.yaml").open(
        encoding="utf-8"
    ) as stream:
        sides = yaml.safe_load(stream)
    with Path(system['robot']['calibration_file']).open(encoding='utf-8') as stream:
        kinematics_hash = str(yaml.safe_load(stream)['kinematics']['hash'])

    underside = float(
        world["fixed_work_surface"]["acrylic_bottom_world_z_m"]
    )
    corner_x, lower_edge_y = acrylic_inner_lower_left_world_xy_m(world)
    clearance_limits = load_clearance_limits_m(root)
    ceiling_clearance = clearance_limits["ceiling"]
    left_guard = float(sides["left_inner_x_m"]) + clearance_limits["left"]
    right_guard = float(sides["right_inner_x_m"]) - clearance_limits["right"]
    if (not math.isfinite(underside) or underside <= ceiling_clearance
            or not math.isfinite(left_guard) or not math.isfinite(right_guard)
            or left_guard >= right_guard):
        raise ValueError("global clearance policy is not physically valid")

    tcp = [float(value) for value in system["robot"]["magnet_tcp_xyz_m"]]
    radius = float(system["robot"]["magnet_cylinder_radius_m"])
    length = float(system["robot"]["magnet_cylinder_length_m"])
    magnet_axis = [float(value) for value in system["robot"]["magnet_axis_tool_vector"]]
    motor_axis = [
        float(value) for value in system["robot"]["motor_axis_tool_vector"]
    ]
    if (len(tcp) != 3 or len(motor_axis) != 3 or len(magnet_axis) != 3
            or radius <= 0.0 or length <= 0.0
            or not all(math.isfinite(v) for v in [*tcp, *motor_axis, *magnet_axis, radius, length])):
        raise ValueError("invalid magnetic tool geometry in ur3_system.yaml")
    axis_norm = math.sqrt(sum(value * value for value in motor_axis))
    magnet_axis_norm = math.sqrt(sum(value * value for value in magnet_axis))
    if axis_norm <= 0.0 or magnet_axis_norm <= 0.0:
        raise ValueError("tool axes must be nonzero")
    motor_axis = [value / axis_norm for value in motor_axis]
    magnet_axis = [value / magnet_axis_norm for value in magnet_axis]
    if abs(sum(a * b for a, b in zip(motor_axis, magnet_axis))) > 1.0e-6:
        raise ValueError("magnet axis must be perpendicular to the motor shaft")

    transform = world["T_base_from_world"]
    if len(transform) != 4 or any(len(row) != 4 for row in transform):
        raise ValueError("T_base_from_world must be a 4x4 matrix")

    return {
        "underside_world_z_m": underside,
        "clearance_limits_m": clearance_limits,
        "guard_world_z_m": underside - ceiling_clearance,
        "acrylic_inner_lower_left_world_xy_m": [corner_x, lower_edge_y],
        "ceiling_region_min_world_y_m": lower_edge_y,
        "table_geometry": world,
        "left_guard_world_x_m": left_guard,
        "right_guard_world_x_m": right_guard,
        "T_base_from_world": [[float(value) for value in row] for row in transform],
        "world_axes_in_base": {
            key: [float(value) for value in world["world_axes_in_base"][key]]
            for key in ("x", "y", "z")
        },
        "magnet_tcp_xyz_m": tcp,
        "magnet_cylinder_radius_m": radius,
        "magnet_cylinder_length_m": length,
        "magnet_axis_tool_vector": magnet_axis,
        "magnet_swept_radius_m": math.hypot(radius, length / 2),
        "magnet_swept_length_m": 2 * radius,
        "motor_axis_tool_vector": motor_axis,
        "motor_axis_verified": system["robot"].get("motor_axis_verified") is True,
        "kinematics_hash": kinematics_hash,
    }


def rotation_matrix_to_quaternion(matrix):
    """Return ROS-order (x, y, z, w) for a proper 3x3 rotation matrix."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def transform_point(transform, point):
    return [
        sum(transform[row][column] * point[column] for column in range(3))
        + transform[row][3]
        for row in range(3)
    ]


class AcrylicCeilingGuard:
    """Apply the global clearance scene and validate robot states against it."""

    def __init__(self, node):
        self.node = node
        self.configuration = load_guard_configuration()
        self.apply_client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.validity_client = node.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.description_client = node.create_client(GetParameters, '/move_group/get_parameters')

    @property
    def world_axes_in_base(self):
        return self.configuration["world_axes_in_base"]

    def _world_box(self, object_id, center_world, dimensions):
        transform = self.configuration["T_base_from_world"]
        center_base = transform_point(transform, center_world)
        rotation = [row[:3] for row in transform[:3]]
        quaternion = rotation_matrix_to_quaternion(rotation)

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(dimensions)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center_base
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = quaternion

        collision = CollisionObject()
        collision.header.frame_id = "base"
        collision.id = object_id
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        return collision

    def _forbidden_ceiling_object(self):
        # The lower Z face is the guarded underside. The lower Y face is the
        # measured acrylic edge; Y below it remains outside this object.
        box_x_m = 2.0
        box_y_m = CEILING_REGION_SPAN_M
        box_height_m = 1.0
        lower_y = self.configuration["ceiling_region_min_world_y_m"]
        return self._world_box(
            CEILING_OBJECT_ID,
            [0.0, lower_y + box_y_m / 2.0,
             self.configuration["guard_world_z_m"] + box_height_m / 2.0],
            [box_x_m, box_y_m, box_height_m],
        )

    def _forbidden_side_objects(self):
        # The inner X faces encode the global 10 mm side margin. The boxes are
        # deliberately larger than the UR3 work envelope in world Y and Z.
        width_m = 1.0
        extent_yz_m = 2.0
        left = self.configuration["left_guard_world_x_m"]
        right = self.configuration["right_guard_world_x_m"]
        return [
            self._world_box(
                LEFT_OBJECT_ID,
                [left - width_m / 2.0, 0.0, 0.5],
                [width_m, extent_yz_m, extent_yz_m],
            ),
            self._world_box(
                RIGHT_OBJECT_ID,
                [right + width_m / 2.0, 0.0, 0.5],
                [width_m, extent_yz_m, extent_yz_m],
            ),
        ]

    def _attached_magnet_cylinder(self):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [
            self.configuration["magnet_swept_length_m"],
            self.configuration["magnet_swept_radius_m"],
        ]
        pose = Pose()
        (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ) = self.configuration["magnet_tcp_xyz_m"]
        axis = self.configuration["motor_axis_tool_vector"]
        quaternion = [-axis[1], axis[0], 0.0, 1.0 + axis[2]]
        if math.hypot(*quaternion) < 1.0e-12:
            quaternion = [1.0, 0.0, 0.0, 0.0]
        norm = math.hypot(*quaternion)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = [value / norm for value in quaternion]

        collision = CollisionObject()
        collision.header.frame_id = "tool0"
        collision.id = MAGNET_OBJECT_ID
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD

        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object = collision
        attached.touch_links = ["tool0", "flange", "wrist_3_link"]
        return attached

    def apply(self, robot_state):
        parameters = GetParameters.Request()
        parameters.names = ['robot_description']
        model = self.node.call(self.description_client, parameters).values[0].string_value
        if self.configuration['kinematics_hash'] not in model:
            raise RuntimeError('MoveIt model is not calibrated for this UR3; use robot/calibrated_moveit.launch.py')
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [
            self._forbidden_ceiling_object(),
            *self._forbidden_side_objects(),
        ]
        request.scene.robot_state = copy.deepcopy(robot_state)
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects = [
            self._attached_magnet_cylinder()
        ]
        response = self.node.call(self.apply_client, request, timeout=10.0)
        if not response.success:
            raise RuntimeError("MoveIt rejected the global clearance planning scene")

    def decorate_state(self, robot_state):
        """Return a state that explicitly retains the guarded attached tool."""
        guarded = copy.deepcopy(robot_state)
        guarded.is_diff = True
        guarded.attached_collision_objects = [self._attached_magnet_cylinder()]
        return guarded

    def validate_state(self, robot_state, label):
        request = GetStateValidity.Request()
        request.robot_state = self.decorate_state(robot_state)
        request.group_name = ""
        response = self.node.call(self.validity_client, request, timeout=5.0)
        if response.valid:
            return
        contacts = []
        for contact in response.contacts[:4]:
            contacts.append(
                f"{contact.contact_body_1}<->{contact.contact_body_2} "
                f"depth={contact.depth * 1000.0:.3f}mm "
                f"at_frame={contact.header.frame_id} ({contact.position.x * 1000.0:.1f},"
                f"{contact.position.y * 1000.0:.1f},"
                f"{contact.position.z * 1000.0:.1f})mm"
            )
        detail = ", ".join(contacts) if contacts else "collision or constraint violation"
        raise RuntimeError(f"clearance guard rejected {label}: {detail}")

    def status_text(self):
        values = self.configuration
        limits = values["clearance_limits_m"]
        return (
            f"ACRYLIC_UNDERSIDE_WORLD_Z={values['underside_world_z_m'] * 1000.0:.3f}mm "
            f"GUARD_Z={values['guard_world_z_m'] * 1000.0:.3f}mm "
            f"ACRYLIC_LOWER_EDGE_Y={values['ceiling_region_min_world_y_m'] * 1000.0:.3f}mm "
            f"BOTTOM_CLEARANCE={limits['ceiling'] * 1000.0:.3f}mm "
            f"SIDE_CLEARANCE={limits['left'] * 1000.0:.3f}mm"
        )
