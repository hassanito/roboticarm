#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import json

class ArmTestPatterns(Node):
    def __init__(self):
        super().__init__('arm_test_patterns')
        
        # Define joint limits
        self.joint_limits = {
            1: {"min": 0, "max": 180, "name": "Base Rotation"},
            2: {"min": 20, "max": 70, "name": "Shoulder"},
            3: {"min": 95, "max": 170, "name": "Elbow"},
            4: {"min": 20, "max": 100, "name": "Gripper"}
        }
        
        # Create publisher for arm commands
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            'arm_command',
            10
        )
        
        self.get_logger().info('Arm Test Patterns Node Started')

    def validate_positions(self, positions):
        """Validate and clamp positions to joint limits"""
        valid_positions = []
        for i, pos in enumerate(positions):
            joint_num = i + 1
            limits = self.joint_limits[joint_num]
            clamped_pos = float(max(limits["min"], min(limits["max"], float(pos))))
            if clamped_pos != pos:
                self.get_logger().warn(
                    f'{limits["name"]}: Position {pos} clamped to {clamped_pos}'
                )
            valid_positions.append(clamped_pos)
        return valid_positions

    def send_position(self, positions, delay=2.0, steps=10):
        """
        Send a position command with smooth movement
        
        Args:
            positions: Target positions for each joint
            delay: Total time to reach position
            steps: Number of intermediate steps
        """
        valid_positions = self.validate_positions(positions)
        
        # Get current positions
        current_positions = self.current_positions if hasattr(self, 'current_positions') else [90.0] * 4
        
        # Calculate step sizes for each joint
        step_sizes = [(target - current) / steps 
                    for current, target in zip(current_positions, valid_positions)]
        
        # Move in steps
        for i in range(steps):
            intermediate_positions = [
                current + (step * i)
                for current, step in zip(current_positions, step_sizes)
            ]
            
            msg = Float64MultiArray()
            msg.data = [float(pos) for pos in intermediate_positions]
            self.command_pub.publish(msg)
            time.sleep(delay/steps)
        
        # Final position
        msg = Float64MultiArray()
        msg.data = [float(pos) for pos in valid_positions]
        self.command_pub.publish(msg)
        time.sleep(delay/steps)
        
        # Store current positions
        self.current_positions = valid_positions

    def get_center_positions(self):
        """Get center position for each joint"""
        return [
            90.0,  # Base center
            45.0,  # Shoulder center (mid of 0-70)
            127.5, # Elbow center (mid of 85-170)
            60.0   # Gripper center (mid of 20-100)
        ]

    def run_square_pattern(self):
        """Move arm in a square pattern within limits"""
        # Home position (centered)
        self.send_position(self.get_center_positions())
        
        # Square corners - adjusted for limits
        # Front left
        self.send_position([45.0, 20.0, 130.0, 60.0])
        
        # Front right
        self.send_position([135.0, 20.0, 130.0, 60.0])
        
        # Back right
        self.send_position([135.0, 60.0, 160.0, 60.0])
        
        # Back left
        self.send_position([45.0, 60.0, 160.0, 60.0])
        
        # Back to home
        self.send_position(self.get_center_positions())

    def run_pick_and_place(self):
        """Simulate pick and place motion within limits"""
        # Home position
        self.send_position(self.get_center_positions())
        
        # Move to pick position
        self.send_position([45.0, 60.0, 130.0, 20.0])
        
        # Close gripper
        self.send_position([45.0, 60.0, 130.0, 100.0])
        
        # Lift object
        self.send_position([45.0, 35.0, 127.5, 100.0])
        
        # Move to place position
        self.send_position([135.0, 60.0, 130.0, 100.0])
        
        # Open gripper
        self.send_position([135.0, 60.0, 130.0, 20.0])
        
        # Return home
        self.send_position(self.get_center_positions())

    def run_wave(self):
        """Make the arm wave within limits"""
        # Start position - raised arm
        self.send_position([90.0, 20.0, 100.0, 60.0], 1.0)
        
        # Wave left and right
        for _ in range(3):
            self.send_position([45.0, 20.0, 100.0, 60.0], 0.5)
            self.send_position([135.0, 20.0, 100.0, 60.0], 0.5)
        
        # Return home
        self.send_position(self.get_center_positions())

def main(args=None):
    rclpy.init(args=args)
    node = ArmTestPatterns()
    
    try:
        # Wait a moment for everything to initialize
        time.sleep(2)
        
        # Print joint limits
        node.get_logger().info('Joint Limits:')
        for joint_num, limits in node.joint_limits.items():
            node.get_logger().info(
                f'{limits["name"]}: {limits["min"]}° to {limits["max"]}°'
            )
        
        # node.get_logger().info('Running square pattern...')
        # node.run_square_pattern()
        
        # node.get_logger().info('Running pick and place pattern...')
        # node.run_pick_and_place()
        
        node.get_logger().info('Running wave pattern...')
        node.run_wave()
        
        # node.get_logger().info('Test patterns completed!')
        
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()