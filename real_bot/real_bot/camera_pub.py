import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)  # Open the default camera (index 0)

    def capture_and_publish(self):
        ret, frame = self.capture.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    while rclpy.ok():
        camera_publisher.capture_and_publish()

    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
