#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from adafruit_pca9685 import PCA9685
import busio
import board
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Initialize hardware
        i2c = busio.I2C(board.SCL_1, board.SDA_1)
        self.pca = PCA9685(i2c, address=0x40)
        self.pca.frequency = 50
        
        # Joint state publisher
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Command subscriber
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            'arm_command',
            self.command_callback,
            10
        )
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Initialize joint names and positions
        self.joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint', 'gripper_joint']
        self.current_positions = [90.0, 35.0, 160.0, 20.0]  # Default positions in degrees
        
        self.get_logger().info('Arm Controller Node Started')

    def angle_to_pulse(self, angle, servo_num):
        """Convert angle (0-180) to pulse width"""
        if servo_num == 3:  # Gripper servo
            pulse_min = 1000
            pulse_max = 8000
        else:  # Main servos
            pulse_min = 2000
            pulse_max = 9000
        return int(pulse_min + (pulse_max - pulse_min) * angle / 180)

    def set_servo_position(self, channel, angle):
        """Set servo position for given channel and angle"""
        pulse = self.angle_to_pulse(angle, channel)
        self.pca.channels[channel].duty_cycle = pulse
        self.current_positions[channel] = angle

    def command_callback(self, msg):
        """Handle incoming command messages"""
        try:
            if len(msg.data) != 4:
                self.get_logger().error('Invalid command length')
                return
                
            # Update each servo position
            for i, angle in enumerate(msg.data):
                if 0 <= angle <= 180:
                    self.set_servo_position(i, angle)
                else:
                    self.get_logger().warn(f'Invalid angle for joint {i}: {angle}')
                    
        except Exception as e:
            self.get_logger().error(f'Error in command callback: {str(e)}')

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        # Convert degrees to radians for ROS
        msg.position = [pos * 3.14159 / 180.0 for pos in self.current_positions]
        msg.velocity = []
        msg.effort = []
        
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = ArmController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
EEZYbotARM MK2 ROS2 Controller Node

This node provides ROS2 control interface for the EEZYbotARM MK2 robot arm.
It handles servo control through a PCA9685 controller and provides ROS2
topics for commanding the arm and reporting its state.

System Architecture:
------------------
1. Hardware Interface:
   - Communicates with PCA9685 servo controller via I2C
   - Controls 4 servos: base, shoulder, elbow, and gripper
   - Servo angles range: 0-180 degrees

2. ROS2 Interface:
   Publishers:
   - Topic: 'joint_states'
   - Type: sensor_msgs/JointState
   - Frequency: 10Hz
   - Content: Current position of all joints in radians

   Subscribers:
   - Topic: 'arm_command'
   - Type: std_msgs/Float64MultiArray
   - Content: Desired joint angles in degrees [base, shoulder, elbow, gripper]

3. Control Flow:
   a. Initialization:
      - Sets up I2C communication
      - Initializes servo controller
      - Creates ROS2 publishers and subscribers
      - Sets default joint positions

   b. Command Processing:
      - Receives commands via 'arm_command' topic
      - Validates angle ranges (0-180 degrees)
      - Converts angles to servo pulses
      - Updates servo positions

   c. State Publishing:
      - Reads current joint positions
      - Converts angles to radians
      - Publishes to 'joint_states' topic
      - Updates at 10Hz

4. Safety Features:
   - Angle range validation
   - Error logging
   - Exception handling

Usage:
------
1. Send commands:
   ros2 topic pub /arm_command std_msgs/Float64MultiArray \
   "data: [90.0, 90.0, 90.0, 90.0]"

2. Monitor joint states:
   ros2 topic echo /joint_states

Note: All angles in commands should be in degrees (0-180).
      Joint states are published in radians (ROS2 standard).

#  Test commandsCenter all joints
ros2 topic pub --once /arm_command std_msgs/msg/Float64MultiArray "{data: [90.0, 90.0, 90.0, 90.0]}"

"""