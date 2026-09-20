import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    """Publish a safe test image, or a configured GStreamer camera pipeline."""

    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameter("mode", "synthetic")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("gstreamer_pipeline", "")
        self.mode = str(self.get_parameter("mode").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        fps = max(1.0, float(self.get_parameter("fps").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "/camera/image_raw", 5)
        self.capture = None
        self.frame_index = 0

        if self.mode == "gstreamer":
            pipeline = str(self.get_parameter("gstreamer_pipeline").value)
            if not pipeline:
                raise RuntimeError("gstreamer mode requires gstreamer_pipeline")
            self.capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not self.capture.isOpened():
                raise RuntimeError("unable to open configured GStreamer pipeline")

        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info(f"camera mode={self.mode}, rate={fps:.1f} Hz")

    def publish_frame(self):
        if self.capture is None:
            image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            image[:] = (28, 28, 28)
            phase = self.frame_index / 120.0 * 2.0 * math.pi
            x = int(self.width * (0.50 + 0.30 * math.sin(phase)))
            y = int(self.height * 0.50)
            cv2.circle(image, (x, y), 24, (0, 255, 0), -1)
            cv2.putText(
                image,
                "SYNTHETIC - NO REAL CAMERA CONTROL",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )
        else:
            ok, image = self.capture.read()
            if not ok:
                self.get_logger().error("camera frame read failed")
                return

        message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)
        self.frame_index += 1

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
