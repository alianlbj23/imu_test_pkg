# IMU Test Package

This ROS 2 package provides a visualization tool for IMU (Inertial Measurement Unit) orientation data. It creates a 3D visualization of IMU orientation using PyBullet.

## Features

- Subscribes to IMU orientation data from the `/imu/data_raw` topic
- Visualizes the orientation as 3D coordinate axes:
  - Red axis: X
  - Green axis: Y
  - Blue axis: Z
- Updates visualization in real-time as IMU data changes
- Simple interface with keyboard control ('q' to quit)

## Hardware Used
This package was developed and tested with the [Yahboom IMU Sensor](https://category.yahboom.net/products/imu) module. This 10DOF IMU sensor integrates gyroscope, accelerometer, magnetometer, and barometer to provide high-precision orientation data.
## Prerequisites

- ROS 2 Humble
- Python 3
- PyBullet (`pip install pybullet==3.2.6`)
- NumPy (`pip install numpy==1.24.3`)
- transforms3d (`pip install transforms3d==0.4.2`)

## Installation

1. Clone this repository into your workspace:
    ```bash
    git clone https://github.com/yourusername/imu_test.git
    ```
2. Build the package:
    ```bash
    cd imu_test_pkg/
    colcon build
    . ./install/setup.bash
    ```

## Usage
1. Start the IMU visualization node:
    ```bash
    ros2 run imu_test_pkg imu_gui_node
    ```

## Troubleshooting
- If the visualization doesn't appear, check that your IMU is publishing to the correct topic (`/imu/data_raw`)