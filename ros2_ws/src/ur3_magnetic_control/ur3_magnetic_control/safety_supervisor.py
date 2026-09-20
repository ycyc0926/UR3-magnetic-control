import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class SafetySupervisor(Node):
    """Watch message freshness and publish a fail-safe stop request."""

    def __init__(self):
        super().__init__("safety_supervisor")
        self.declare_parameter("target_timeout_s", 0.2)
        self.declare_parameter("robot_state_timeout_s", 0.2)
        self.declare_parameter("require_robot_state", False)
        self.target_timeout = float(self.get_parameter("target_timeout_s").value)
        self.robot_timeout = float(self.get_parameter("robot_state_timeout_s").value)
        self.require_robot_state = bool(self.get_parameter("require_robot_state").value)
        self.last_target = None
        self.last_robot_state = None
        self.target_subscription = self.create_subscription(
            PoseStamped, "/origami/pose_pixels", self.on_target, 10
        )
        self.robot_subscription = self.create_subscription(
            JointState, "/joint_states", self.on_robot_state, 10
        )
        self.stop_publisher = self.create_publisher(
            Bool, "/safety/stop_requested", 10
        )
        self.status_publisher = self.create_publisher(String, "/safety/status", 10)
        self.timer = self.create_timer(0.05, self.check)

    def on_target(self, _message):
        self.last_target = time.monotonic()

    def on_robot_state(self, _message):
        self.last_robot_state = time.monotonic()

    def check(self):
        now = time.monotonic()
        reasons = []
        if self.last_target is None or now - self.last_target > self.target_timeout:
            reasons.append("target_stale")
        if self.require_robot_state and (
            self.last_robot_state is None
            or now - self.last_robot_state > self.robot_timeout
        ):
            reasons.append("robot_state_stale")
        stop = Bool()
        stop.data = bool(reasons)
        self.stop_publisher.publish(stop)
        status = String()
        status.data = "STOP:" + ",".join(reasons) if reasons else "READY"
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
