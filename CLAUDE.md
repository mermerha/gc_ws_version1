# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a **ROS (Robot Operating System) catkin workspace** for an autonomous UAV system used in a competition (工创赛). The drone uses PX4 flight controller, Livox AVIA LiDAR for mapping, and EGO-Planner for trajectory planning.

## Build & Run

```bash
# Build the entire workspace (from workspace root)
catkin_make

# Source the workspace after building
source devel/setup.bash

# Build a single package
catkin_make --pkg <package_name>
```

## Key Launch Scripts (in `shfiles/`)

| Script | Purpose |
|--------|---------|
| `run_gc.sh` | Full system startup: mavros → faster-lio → px4ctrl → ego_planner |
| `run_demo.sh` | Same as run_gc.sh (competition demo) |
| `run_planning.sh` | Launch ego_planner only (interactive waypoint mode) |
| `run_vis.sh` | Launch RViz visualization only |
| `takeoff.sh` | Launch mission control node (starts mission state machine) |
| `land.sh` | Send land command via rostopic |
| `mavros.sh` | Start mavros with message rate configuration |

## System Architecture

### Data Flow
```
Livox AVIA LiDAR + IMU
        ↓
   faster-lio (LiDAR odometry → /Odometry, /cloud_registered)
        ↓
  ekf_pose (EKF fusion with IMU → /ekf/ekf_odom)
        ↓
  ego_planner (trajectory planning, subscribes /ekf/ekf_odom + /cloud_registered)
        ↓
  traj_server (publishes /setpoints_cmd)
        ↓
  px4ctrl (position controller → mavros → PX4 FCU)
```

### Key Packages

**`src/faster-lio/`** — LiDAR-Inertial Odometry
- Based on FastLIO2, ~1.5-2x faster
- Main launch: `mapping_with_driver.launch` (uses Livox AVIA with onboard driver)
- Config: `config/avia.yaml`
- Publishes: `/Odometry` (pose), `/cloud_registered` (point cloud)

**`src/ekf_pose/`** — EKF State Estimator
- Fuses IMU (`/mavros/imu/data`) with LiDAR odometry or mocap
- Outputs: `/ekf/ekf_odom`
- Variants: `ekf_node.cpp` (mocap), `ekf_node_vio_timesync.cpp` (LiDAR/VIO with time sync + outlier rejection)
- Launch configs: `mocap_and_lidar.launch`, `nokov.launch`

**`src/planner/`** — EGO-Planner (trajectory planning)
- `plan_manage/` — top-level: `ego_replan_fsm.cpp` (state machine), `planner_manager.cpp`
- `path_searching/` — dynamic A* (`dyn_a_star.cpp`)
- `plan_env/` — environment representation (occupancy grid)
- `traj_opt/` — trajectory optimization
- `traj_utils/` — trajectory math utilities
- `swarm_bridge/` — multi-drone TCP communication
- `drone_detect/` — detects other drones in depth image
- Main exp launch: `plan_manage/launch/run_in_exp_interactive.launch`
  - `flight_type=1`: interactive (2D Nav Goal in RViz)
  - `flight_type=2`: autonomous waypoints (configure `point_num` and `pointN_*`)

**`src/traj_actor/px4ctrl/`** — PX4 Position Controller
- Implements cascaded PID with feedforward
- Subscribes: `/ekf/ekf_odom`, `/setpoints_cmd`
- Communicates with PX4 via mavros (OFFBOARD mode)
- Launch: `run_ctrl.launch`

**`src/traj_actor/traj_server/`** — Trajectory Server
- Converts planned trajectory to position setpoints
- Publishes `/setpoints_cmd`

**`src/mission_control/`** — Mission State Machine
- Handles high-level competition task logic (takeoff → navigate waypoints → detect color → pick/drop ball → land)
- Config: `config/mission_params_new.yaml` (all waypoints and heights)
- State machine enum in `src/*.cpp`: TAKING_OFF → TO_POINT_N → CATCH_BALL → IDENTIFY_COLOR → DROP_BALL → LANDING
- Subscribes: `/ekf/ekf_odom`, color detection service `/detect_color`
- Publishes: `/goal_with_id` (goals to ego_planner), `/px4ctrl/takeoff_land`, `/catch`, `/release`

**`src/ekf_pose/`** and **`src/Utils/`** — Utilities
- `Utils/` contains: `quadrotor_msgs`, `uav_utils`, `odom_visualization`, `pose_utils`, `assign_goals`, `moving_obstacles`, `random_goals`, `rviz_plugins`, `selected_points_publisher`, `manual_take_over`

**`src/livox_ros_driver/`** and **`src/livox_ros_driver2/`** — Livox LiDAR ROS drivers

**`src/uav_simulator/`** — Simulation components (fake_drone, local_sensing, map_generator, so3_control, so3_quadrotor_simulator)

## Key ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ekf/ekf_odom` | nav_msgs/Odometry | Fused pose estimate (primary odom) |
| `/cloud_registered` | sensor_msgs/PointCloud2 | LiDAR point cloud from faster-lio |
| `/Odometry` | nav_msgs/Odometry | Raw LiDAR odometry from faster-lio |
| `/setpoints_cmd` | quadrotor_msgs/PositionCommand | Position setpoints to px4ctrl |
| `/goal_with_id` | quadrotor_msgs/GoalSet | Navigation goals to planner |
| `/px4ctrl/takeoff_land` | quadrotor_msgs/TakeoffLand | Takeoff/land commands |

## Configuration Files

- **EGO-Planner params**: `src/planner/plan_manage/launch/advanced_param_exp.xml`
- **Mission waypoints**: `src/mission_control/config/mission_params_new.yaml`
- **LiDAR mapping**: `src/faster-lio/config/avia.yaml`
- **EKF params**: `src/ekf_pose/launch/PX4_vio_drone.yaml`
- **px4ctrl params**: `src/traj_actor/px4ctrl/config/`

## Livox LiDAR Serial Number

The configured Livox AVIA serial number in `mapping_with_driver.launch` is `47MDKAA0010912`. Update this if using a different unit.

## MAVLink Message Rates (set by mavros.sh)
- IMU raw (msg 27): 333 Hz
- Attitude (msg 30): 333 Hz  
- Local position (msg 32, ID 65): 100 Hz
- Actuator output (msg 36): 333 Hz
