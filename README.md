# trajectory_generator

ROS 2 node that generates reference trajectories for ground robots. Supports circle, figure-8, and line (back-and-forth) shapes, published as `nav_msgs/msg/Path`.

## Dependencies

- ROS 2 Humble
- `rclcpp`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2`, `tf2_geometry_msgs`
- Eigen3 (`eigen3_cmake_module`)

## Build

```bash
cd ~/trajectory_generator_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select trajectory_generator
source install/setup.bash
```

## Run

```bash
# Standalone (requires traj_shape argument)
ros2 launch trajectory_generator trajectory_generator.launch.py traj_shape:=8       # figure-8
ros2 launch trajectory_generator trajectory_generator.launch.py traj_shape:=circle  # circle
ros2 launch trajectory_generator trajectory_generator.launch.py traj_shape:=line    # line
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `traj_shape` | (required) | `8`, `circle`, or `line` |
| `robot_name` | `$ROVER_NAME` or `RR03` | Namespace for all topics |

## Published topics

All topics are relative (namespaced by `robot_name`):

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `reference_trajectory` | `nav_msgs/msg/Path` | reliable + transient_local | Reference path, re-published at 1 Hz |
| `trajectory_markers` | `visualization_msgs/msg/MarkerArray` | default | RViz visualization markers |

## Parameters

Configured via `config/params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pub_freq` | `10.0` | Publish frequency (Hz) |
| **Circle** | | |
| `r` | `3.0` | Radius (m) |
| `center_x`, `center_y` | `0.0` | Circle center in odom frame |
| `v_goals` | `[0.3, 0.5, 0.8, 1.0]` | Target velocities per segment (m/s) |
| `t_traj` | `20.0` | Trajectory duration (s) |
| `circle_accel` | `0.3` | Acceleration (m/s^2) |
| **Figure-8** | | |
| `amplitude_x` | `5.0` | Half-width in X (m) |
| `amplitude_y` | `3.0` | Half-height in Y (m) |
| `num_points` | `200` | Number of waypoints |
| **Line** | | |
| `Ax`, `Ay` | `0.0, 0.0` | Start point A (m) |
| `Bx`, `By` | `8.0, 0.0` | End point B (m) |
| `v_line` | `0.8` | Cruise velocity (m/s) |
| `num_laps` | `20` | Number of round trips (A->B->A = 1 lap) |
