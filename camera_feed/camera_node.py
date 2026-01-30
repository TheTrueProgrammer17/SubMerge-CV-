# camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1/30.0, self.capture_and_publish)

        GST_PIPELINE = (
            "v4l2src device=/dev/video0 ! "
            "video/x-raw,width=640,height=480,framerate=30/1 ! "
            "videoconvert ! appsink drop=1"
        )
        self.cap = cv.VideoCapture(GST_PIPELINE, cv.CAP_GSTREAMER)

    def capture_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame")
            return

        # Apply underwater filters
        lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
        l, a, b = cv.split(lab)
        clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        l = clahe.apply(l)
        lab = cv.merge((l,a,b))
        frame = cv.cvtColor(lab, cv.COLOR_LAB2BGR)

        # Boost reds
        b, g, r = cv.split(frame)
        r = cv.addWeighted(r, 1.5, g, -0.2, 0)
        frame = cv.merge((b, g, r))

        # Gamma correction
        gamma = 1.2
        lookup = np.array([((i / 255.0) ** (1.0 / gamma)) * 255 for i in range(256)]).astype("uint8")
        frame = cv.LUT(frame, lookup)

        # Publish to ROS2
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

        # Optional: display window
        cv.imshow("Underwater Camera Feed", frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
