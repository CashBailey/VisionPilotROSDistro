# VisionPilotROSDistro
Position-based visual servoing (PBVS) to guide a drone using camera image

## ROS2 Migration

A partial ROS&nbsp;2 Jazzy Jalisco port lives in the `ROS2` directory. It currently contains Python implementations of the PBVS and offboard control nodes using `rclpy`.

Build the ROS2 packages with `colcon build` inside a ROS2 workspace:

```bash
colcon build --packages-select ros2_offboard
```

To start the PBVS and offboard nodes together use the provided launch file:

```bash
ros2 launch ros2_offboard start_offb.launch.py
```
