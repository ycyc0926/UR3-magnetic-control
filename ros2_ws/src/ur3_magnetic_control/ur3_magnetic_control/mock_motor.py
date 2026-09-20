import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class MockMotor(Node):
    """Motor placeholder that refuses nonzero output unless explicitly enabled."""

    def __init__(self):
        super().__init__("mock_motor")
        self.declare_parameter("allow_nonzero", False)
        self.declare_parameter("max_rpm", 0.0)
        self.allow_nonzero = bool(self.get_parameter("allow_nonzero").value)
        self.max_rpm = max(0.0, float(self.get_parameter("max_rpm").value))
        self.state_rpm = 0.0
        self.command_subscription = self.create_subscription(
            Float64, "/motor/command_rpm", self.on_command, 10
        )
        self.state_publisher = self.create_publisher(
            Float64, "/motor/state_rpm", 10
        )
        self.available_publisher = self.create_publisher(
            Bool, "/motor/available", 10
        )
        self.timer = self.create_timer(0.05, self.publish_state)
        self.get_logger().info(
            f"mock motor; allow_nonzero={self.allow_nonzero}, max_rpm={self.max_rpm:.1f}"
        )

    def on_command(self, message):
        requested = float(message.data)
        if not self.allow_nonzero or self.max_rpm <= 0.0:
            if abs(requested) > 1e-9:
                self.get_logger().warning("nonzero command rejected by safety gate")
            self.state_rpm = 0.0
            return
        self.state_rpm = max(-self.max_rpm, min(self.max_rpm, requested))

    def publish_state(self):
        state = Float64()
        state.data = self.state_rpm
        self.state_publisher.publish(state)
        available = Bool()
        available.data = False
        self.available_publisher.publish(available)


def main(args=None):
    rclpy.init(args=args)
    node = MockMotor()
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
