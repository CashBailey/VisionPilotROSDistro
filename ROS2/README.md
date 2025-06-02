# ROS2 Port

This folder contains the beginnings of a ROS&nbsp;2 Jazzy Jalisco migration for the original ROS&nbsp;1 workspace.

## Offboard Package

`ros2_offboard` provides Python implementations of the `pbvs_node` and `offb_node` using `rclpy` and the ROS&nbsp;2 build system (`ament_python`).

Build the package inside a ROS&nbsp;2 workspace:

```bash
colcon build --packages-select ros2_offboard
```

Run the PBVS node:

```bash
ros2 run ros2_offboard pbvs_node
```

To run both the PBVS and offboard nodes together:

```bash
ros2 launch ros2_offboard start_offb.launch.py
```

The node expects the same reference image path used by the ROS&nbsp;1 version but can be overridden via the `reference_image` parameter.

This is only a partial port intended as a starting point for migrating the rest of the repository.
