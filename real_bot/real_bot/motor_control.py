
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel_auto', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel_auto', self.callback, 10)
        
        # Change the serial port based on your Arduino connection
        self.arduino_serial = serial.Serial('/dev/ttyUSB0', 9600)
        
        self.get_logger().info("Motor Controller Node Initialized")

    def callback(self, msg):
        self.get_logger().info("Received Twist command")

        # Publish the received Twist message
        self.publisher.publish(msg)
        
        # Convert linear and angular velocities to motor speeds
        #right_wheel_vel = ( msg.linear.x  + msg.angular.z ) /2
        #left_wheel_vel = (  msg.linear.x  - msg.angular.z ) /2
        right_wheel_vel = int(msg.linear.x * 255) 
        left_wheel_vel = int(msg.angular.z * 255)
        #right_wheel_vel = ( msg.linear.x)
        #left_wheel_vel = (  msg.linear.x)
        # Create the command string for Arduino
        command = f'{right_wheel_vel},{left_wheel_vel}\n'
        
        # Send the command to Arduino
        self.arduino_serial.write(command.encode())
        self.get_logger().info(f"Sent command to Arduino: {command}")

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
