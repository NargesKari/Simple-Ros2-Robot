# Simple-Ros2-Robot

A differential-drive robot simulated in Gazebo with a ROS 2 pose-estimation pipeline: IMU low-pass filtering, bias correction, complementary-filter orientation, and wheel-odometry fused into a live `odom -> base_link` transform.

## What it does

- URDF/Xacro model of a differential-drive robot (cylindrical base, continuous-joint wheels, 2D LiDAR, IMU) simulated in Gazebo via `ros_gz_bridge`.
- `lowpass_imu_node` filters raw IMU noise and applies bias correction before anything downstream consumes it.
- `complementary_filter_node` fuses the filtered IMU into an orientation estimate.
- `motion_controller_node` converts `/cmd_vel` into left/right wheel RPM commands.
- `odometry_publisher_node` turns wheel motion into odometry and broadcasts the `odom -> base_link` TF.
- `frame_id_converter` standardizes the simulated LiDAR's frame ID/topic to match ROS 2 conventions.

## Tech stack

ROS 2 (`rclpy`), Gazebo (`ros_gz_bridge`), URDF/Xacro, RViz.

## Getting started

```bash
colcon build --packages-select robot_description robot_estimation
source install/setup.bash

ros2 launch robot_description gazebo.launch.py     # spawn the robot + world + RViz
ros2 launch robot_estimation estimator.launch.py   # start the filtering/odometry nodes
```

`robot_description display.launch.py` shows the model alone in RViz without Gazebo, useful for checking the URDF and TF tree before running the full simulation.
