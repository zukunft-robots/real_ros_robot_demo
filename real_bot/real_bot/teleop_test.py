import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.callback, 10)
        self.arduino_serial = serial.Serial('/dev/ttyUSB0', 9600)
        self.get_logger().info("Motor Controller Node Initialized")

    def callback(self, msg):
        
        self.publisher_.publish(msg)
        
        
        rightwheelvel = int(msg.linear.x * 255) 
        leftwheelvel = int(msg.angular.z * 255) 
        
        
        command = f'{rightwheelvel},{leftwheelvel}\n' 
        self.arduino_serial.write(command.encode())
        self.get_logger().info(f"Sent command to Arduino: {command}")

def main(args=None):
    rclpy.init(args=args)
    robot_controller = MotorController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()