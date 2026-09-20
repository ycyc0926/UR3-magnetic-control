import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")
        self.declare_parameter("length_m", 0.10)
        self.declare_parameter("point_count", 51)
        self.declare_parameter("frame_id", "workspace")
        self.length = float(self.get_parameter("length_m").value)
        self.point_count = max(2, int(self.get_parameter("point_count").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publisher = self.create_publisher(Path, "/desired_path", 1)
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for index in range(self.point_count):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = self.length * index / (self.point_count - 1)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
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
