# Dynamic Obstacle Navigation

A ROS 2 Nav2 stack for navigating a TurtleBot3 waffle\_pi through a dynamic environment with moving obstacles. Includes a custom **Dynamic Window Approach (DWA)** local controller implemented as a `nav2_core::Controller` plugin, benchmarked against the stock Nav2 DWB controller.

## Demo



[▶ Gazebo world overview](https://github.com/PBayim/robot_path_planning/issues/5#issue-4761319392)

### Navigation recordings

**DWB (baseline)**

[▶ DWB run 1](https://github.com/PBayim/robot_path_planning/issues/1#issue-4761300985) | [▶ DWB run 2](https://github.com/PBayim/robot_path_planning/issues/2#issue-4761302119)

**Custom DWA**

[▶ Custom DWA run 1](https://github.com/PBayim/robot_path_planning/issues/3#issue-4761303091) | [▶ Custom DWA run 2](https://github.com/PBayim/robot_path_planning/issues/4#issue-4761304308)

## Packages

| Package | Description |
|---|---|
| `tb3_dwa_controller` | Custom C++ DWA controller plugin |
| `tb3_dynamic_nav_bringup` | Launch files, Nav2 params, map, RViz config |
| `tb3_moving_obstacles` | Gazebo world and scripted obstacle mover node |
| `tb3_experiment_logger` | Passive metrics logger that records one CSV row per navigation run |

## Requirements

- ROS 2 Humble
- Nav2 (`ros-humble-navigation2`, `ros-humble-nav2-bringup`)
- TurtleBot3 packages (`ros-humble-turtlebot3`, `ros-humble-turtlebot3-simulations`)
- Gazebo Classic 11 (`ros-humble-gazebo-ros-pkgs`)
- `slam_toolbox` (map was pre-built; only needed if remapping)

## Build

```bash
source setup_env.bash   # sets ROS 2, TURTLEBOT3_MODEL, GAZEBO_MODEL_PATH, workspace
colcon build
source install/setup.bash
```

## Run

### DWB controller (baseline)

```bash
ros2 launch tb3_dynamic_nav_bringup dynamic_nav.launch.py
```

### Custom DWA controller

```bash
ros2 launch tb3_dynamic_nav_bringup dynamic_nav.launch.py nav2_params_file:=$(ros2 pkg prefix tb3_dynamic_nav_bringup)/share/tb3_dynamic_nav_bringup/params/nav2_params_dwa.yaml default_nav_to_pose_bt_xml:=$(ros2 pkg prefix tb3_dynamic_nav_bringup)/share/tb3_dynamic_nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery_dwa.xml
```

> The `$(ros2 pkg prefix ...)` subshells expand at runtime, so this command works on any machine regardless of where the workspace is installed.

Both commands start Gazebo with the dynamic obstacle world, AMCL localisation, Nav2, and RViz2.
The robot auto-initialises at **(−2.0, −0.5, yaw = 0)**, so no manual pose estimate is needed.

Send goals from the **Nav2 Goal** tool in the RViz2 toolbar.

## Logging

The experiment logger runs passively alongside navigation and appends one CSV row to
`results/results.csv` each time a `NavigateToPose` goal reaches a terminal state.
It detects goals via the action status topic, so there is no need to configure goal coordinates separately.

```bash
ros2 run tb3_experiment_logger metrics_logger --ros-args \
  -p controller:=custom_dwa \
  -p output_csv:=$PWD/results/results.csv
```

**CSV fields:** `run`, `timestamp`, `controller`, `goal_x/y/yaw_deg`, `status`,
`time_to_goal_s`, `num_recoveries`, `final_x/y/yaw_deg`, `xy_error_m`, `yaw_error_deg`,
`planned/executed_path_length_m`, `path_length_ratio`,
`mean/max_linear_vel`, `mean/max_linear_accel`, `mean/max_angular_vel`,
`mean/max_tracking_error_m`.

## DWA Controller Parameters

Configured in `src/tb3_dynamic_nav_bringup/params/nav2_params_dwa.yaml` under `FollowPath`:

| Parameter | Value | Description |
|---|---|---|
| `max_vel_x` | 0.22 m/s | Maximum forward speed |
| `max_vel_theta` | 1.0 rad/s | Maximum angular speed |
| `vx_samples` | 10 | Linear velocity samples in the dynamic window |
| `vtheta_samples` | 20 | Angular velocity samples in the dynamic window |
| `sim_time` | 1.5 s | Trajectory rollout horizon |
| `lookahead_distance` | 1.0 m | Local goal look-ahead along the global plan |
| `heading_weight` | 0.8 | Weight for heading-alignment critic |
| `clearance_weight` | 0.3 | Weight for obstacle clearance critic |
| `velocity_weight` | 0.2 | Weight for forward speed critic |
| `goal_slowdown_distance` | 0.5 m | Distance at which the robot begins decelerating to stop |
| `oscillation_reset_dist` | 0.05 m | Oscillation guard (translation reset threshold) |
| `oscillation_reset_angle` | 0.2 rad | Oscillation guard (rotation reset threshold) |

## Moving Obstacles

Two physics-enabled cylinders (radius 0.15 m, height 0.8 m) move along predefined
ping-pong trajectories driven by `obstacle_mover` at 20 Hz:

| Obstacle | Trajectory | Speed |
|---|---|---|
| `moving_obstacle_1` (red) | Horizontal: x ∈ [−1.2, 1.6] m at y = 0.4 m | 0.35 m/s |
| `moving_obstacle_2` (blue) | Vertical: y ∈ [−1.6, 1.3] m at x = 0.6 m | 0.30 m/s |

Trajectories and speed are configured in
`src/tb3_moving_obstacles/config/obstacles.yaml`.

## Repository Layout

```
robot_path_planning/
├── media/                           # Screenshots and screencasts
├── src/
│   ├── tb3_dwa_controller/          # C++ Nav2 controller plugin
│   │   ├── include/
│   │   └── src/dwa_controller.cpp
│   ├── tb3_dynamic_nav_bringup/
│   │   ├── launch/dynamic_nav.launch.py
│   │   ├── maps/turtlebot3_world.yaml
│   │   ├── params/nav2_params.yaml
│   │   └── params/nav2_params_dwa.yaml
│   ├── tb3_moving_obstacles/
│   │   ├── config/obstacles.yaml
│   │   ├── worlds/turtlebot3_dynamic_world.world
│   │   └── tb3_moving_obstacles/obstacle_mover.py
│   └── tb3_experiment_logger/
│       └── tb3_experiment_logger/metrics_logger.py
└── results/
    └── results.csv
```

## License

Apache 2.0
