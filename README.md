# EEZYbotARM MK2 ROS2 Control

ROS2 control system for the EEZYbotARM MK2 3D printed robotic arm using a Jetson Orin Nano and PCA9685 servo controller.

## Overview

This package provides ROS2 nodes for controlling the EEZYbotARM MK2 robotic arm. It includes:
- Basic servo control
- Joint state publishing
- Pre-programmed movement patterns
- Configurable joint limits

### System Architecture

The system consists of two main nodes:
1. **Arm Controller Node**: Core node that handles hardware communication and servo control
2. **Test Patterns Node**: Optional node for testing and demonstration

## Hardware Requirements

- EEZYbotARM MK2 (3D printed)
- NVIDIA Jetson Orin Nano
- PCA9685 servo controller
- 3x MG996R servos (base, shoulder, elbow)
- 1x SG90 servo (gripper)

## Software Requirements

- Ubuntu 22.04 (or compatible OS)
- ROS2 Humble
- Python 3.8+
- Required Python packages:
  - rclpy
  - smbus2
  - adafruit_pca9685
  - busio
  - board

## Installation

1. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone <repository_url> eezybotarm_control
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Starting the Arm Controller

Run the main control node:
```bash
ros2 run eezybotarm_control arm_controller
```

### Running Test Patterns

Run the test patterns node:
```bash
ros2 run eezybotarm_control arm_test_patterns
```

### Using the Launch File

Start both nodes together:
```bash
ros2 launch eezybotarm_control arm_bringup.launch.py
```

### Manual Control

Send commands to move the arm:
```bash
ros2 topic pub --once /arm_command std_msgs/msg/Float64MultiArray \
    "data: [90.0, 45.0, 120.0, 60.0]"
```

## Joint Configuration

Default joint limits:
```json
{
    "1": {"min": 0, "max": 180, "name": "Base Rotation"},
    "2": {"min": 0, "max": 70, "name": "Shoulder"},
    "3": {"min": 85, "max": 170, "name": "Elbow"},
    "4": {"min": 20, "max": 100, "name": "Gripper"}
}
```

## ROS2 Topics

### Published Topics

- `/joint_states` (sensor_msgs/JointState)
  - Current position of all joints
  - Published at 10Hz

### Subscribed Topics

- `/arm_command` (std_msgs/Float64MultiArray)
  - Command joint positions in degrees
  - Array of 4 float values [base, shoulder, elbow, gripper]

## Node Details

### arm_controller.py

Core control node that:
- Manages hardware communication
- Handles servo control
- Publishes joint states
- Receives and executes movement commands

### arm_test_patterns.py

Test node that:
- Provides pre-programmed movement patterns
- Validates movements against joint limits
- Implements smooth motion control
- Includes demo sequences (square, pick-and-place, wave)

## Development

### Building Changes

After modifying code:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select eezybotarm_control
source install/setup.bash
```

### Adding New Features

1. Modify existing nodes or create new ones in the `eezybotarm_control` directory
2. Update `setup.py` if adding new nodes
3. Build and source the workspace

## Troubleshooting

Common issues and solutions:

1. **Servo Movement Issues**
   - Check I2C connections
   - Verify servo power supply
   - Confirm joint limits in configuration

2. **Communication Errors**
   - Check I2C address (default: 0x40)
   - Verify Jetson I2C setup
   - Confirm ROS2 environment setup

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit changes
4. Create a pull request



## Authors

[Hassanito]
