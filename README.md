# Fake Encoder ROS 2 Package

This repository contains a ROS 2 workspace with a `fake_encoder_pkg` package that simulates wheel encoders and provides odometry calculations for robotics applications.

## Package Overview

The `fake_encoder_pkg` contains three main nodes:

1. **Encoder Node** (`fake_encoder.py`) - Simulates wheel encoder ticks
2. **Driver Node** (`encoder_driver.py`) - Processes encoder data
3. **Odometry Node** (`wheel_odom.py`) - Calculates robot odometry from wheel encoders

## Prerequisites

- **ROS 2** (Humble Hawksbill recommended, but compatible with other distributions)
- **Python 3.8+**
- **Setuptools** and **Colcon** build tools

## Installation & Setup

### 1. Clone the Repository

```bash
git clone https://github.com/Tehnomant/Sensors_lab0_ROS.git
cd Sensors_lab0_ROS/ros2_ws
```

### 2. Install Dependencies

```bash
# Install Python dependencies
pip3 install setuptools pytest
```

### 3. Build the Package

```bash
# Source your ROS 2 installation
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select fake_encoder_pkg

# Source the workspace
source install/setup.bash
```

## Launching Nodes

### Launch Individual Nodes

```bash
# Terminal 1: Encoder Node
ros2 run fake_encoder_pkg encoder

# Terminal 2: Driver Node  
ros2 run fake_encoder_pkg driver

# Terminal 3: Odometry Node
ros2 run fake_encoder_pkg odometry
```