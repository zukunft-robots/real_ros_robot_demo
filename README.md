# real_ros_robot_demo
This is a real mobile robot using Raspberry pi



Install UBUNTU & ROS in Raspberry pi


          version: Ros2 Foxy Fitzroy  #refer official ROS2 Documentation



Create a new workspace using the command:


        mkdir -p ~/ros_ws/src



clone the Git repository:


         git clone "URL"

Build the package:


        colcon build

Source the file:


       source install/setup.bash


PACKAGE NAME: real_bot

To run the ROBOT using teleop_twist_keyboard:


         ros2 run real_bot teleop_test_node
 
         ros2 run teleop_twist_keyboard teleop_twist_keyboard

To run the ROBOT using color detection algorithm:
        
        sudo apt update

        sudo apt upgrade

        sudo apt -y install python-cv-bridge    

        sudo  apt-get install python3-opencv


        sudo apt-get install python3-dev python3-numpy


        ros2 run real_bot cam_pub_node                                         #publishing_raspi_camera
  
  
        ros2 run real_bot cam_sub_node                                         #publishing_opencv_algorithm 
  
  
        ros2 run real_bot motor_controller_node                                
  ## serial communication

Write a arduino code for the movement of motors by receiving twist message from ros2 node.


For example:



       const int rightMotorA = 2;      // Connect to motor driver IN1
       const int rightMotorB = 3;      // Connect to motor driver IN2
       const int rightMotorEnable = 6;     // Connect to motor driver ENA

       const int leftMotorA = 4; // Connect to motor driver IN3
       const int leftMotorB = 5; // Connect to motor driver IN4
       const int leftMotorEnable = 9; // Connect to motor driver ENB

       void setup() {
         pinMode(rightMotorA, OUTPUT);
         pinMode(rightMotorB, OUTPUT);
         pinMode(rightMotorEnable, OUTPUT);

         pinMode(leftMotorA, OUTPUT);
         pinMode(leftMotorB, OUTPUT);
         pinMode(leftMotorEnable, OUTPUT);

         // Begin serial communication at 9600 baud rate
         Serial.begin(9600);
        }

       void loop() {


     if (Serial.available() >= 8) {

  
    float linearVel = Serial.parseFloat();
    float angularVel = Serial.parseFloat();

    // Convert twist velocities into motor speeds
    float rightMotorSpeed = linearVel + (angularVel / 2);
    float leftMotorSpeed = linearVel - (angularVel / 2);

    // Control right motor
    if (rightMotorSpeed > 0) {
      digitalWrite(rightMotorA, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorEnable, abs(rightMotorSpeed) * 255);
    } else if (rightMotorSpeed < 0) {
      digitalWrite(rightMotorA, LOW);
      digitalWrite(rightMotorB, HIGH);
      analogWrite(rightMotorEnable, abs(rightMotorSpeed) * 255);
    } else {
      digitalWrite(rightMotorA, LOW);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorEnable, 0); // Stop motor
    }

    // Control left motor
    if (leftMotorSpeed > 0) {
      digitalWrite(leftMotorA, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorEnable, abs(leftMotorSpeed) * 255);
    } else if (leftMotorSpeed < 0) {
      digitalWrite(leftMotorA, LOW);
      digitalWrite(leftMotorB, HIGH);
      analogWrite(leftMotorEnable, abs(leftMotorSpeed) * 255);
    } else {
      digitalWrite(leftMotorA, LOW);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorEnable, 0); // Stop motor
    }
  }
}
