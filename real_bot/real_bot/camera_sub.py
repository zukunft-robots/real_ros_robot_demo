import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.bridge = CvBridge()
        self.get_logger().info("Autonomous Controller Node Initialized")

    def image_callback(self, msg):
        self.get_logger().info("Received camera image")

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert BGR image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red color in HSV
        lower_red = np.array([0 , 100 , 100])  # Adjust these values based on your specific shade of red
        upper_red = np.array([10 , 255 , 255])

        # Threshold the image to get red color regions
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Count the number of red pixels
        red_pixel_count = cv2.countNonZero(red_mask)

        # Print a message if a threshold number of red pixels is detected
        threshold = 1000  # Adjust the threshold as needed
        if red_pixel_count > threshold:
            self.get_logger().info("Red color detected!")
            # Move the robot forward
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.2  # Linear velocity for forward movement
            cmd_vel_msg.angular.z = 0.0  # Angular velocity for rotation
            self.publisher.publish(cmd_vel_msg)
        else:
            stop_cmd_vel_msg = Twist()
            self.publisher.publish(stop_cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    autonomous_controller = AutonomousController()
    autonomous_controller.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)  # Set log level to INFO
    autonomous_controller.get_logger().info("Starting Autonomous Controller Node")
    rclpy.spin(autonomous_controller)
    autonomous_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
