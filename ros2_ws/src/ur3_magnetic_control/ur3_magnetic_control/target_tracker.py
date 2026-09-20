import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class TargetTracker(Node):
    """Minimal green-target tracker for validating the vision message path."""

    def __init__(self):
        super().__init__("target_tracker")
        self.declare_parameter("minimum_area_px", 200.0)
        self.minimum_area = float(self.get_parameter("minimum_area_px").value)
        self.bridge = CvBridge()
        self.pose_publisher = self.create_publisher(
            PoseStamped, "/origami/pose_pixels", 10
        )
        self.confidence_publisher = self.create_publisher(
            Float32, "/origami/confidence", 10
        )
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.on_image, 5
        )

    def on_image(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (35, 80, 60), (90, 255, 255))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        confidence = Float32()
        if not contours:
            self.confidence_publisher.publish(confidence)
            return
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.minimum_area:
            self.confidence_publisher.publish(confidence)
            return
        moments = cv2.moments(contour)
        if moments["m00"] == 0.0:
            return

        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = "camera_pixels"
        pose.pose.position.x = moments["m10"] / moments["m00"]
        pose.pose.position.y = moments["m01"] / moments["m00"]
        pose.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose)
        image_area = float(image.shape[0] * image.shape[1])
        confidence.data = min(
            1.0, area / max(self.minimum_area * 4.0, image_area * 0.01)
        )
        self.confidence_publisher.publish(confidence)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTracker()
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
