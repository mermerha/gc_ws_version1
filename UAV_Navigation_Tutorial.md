# 无人机自主导航系统使用教程

## 一、驱动包概览

整个系统由以下几个驱动/感知包支撑，了解它们的作用有助于排查问题。

### 1. Livox 雷达驱动（`livox_ros_driver2`）
- 驱动 Livox AVIA 固态激光雷达，将原始雷达数据发布为 ROS 点云消息
- 一般不需要修改，但每台雷达有唯一序列号，需要在 `src/faster-lio/launch/mapping_with_driver.launch` 里确认配置正确：
  ```xml
  <arg name="bd_list" default="47MDKAA0010912"/>
  ```
  如果换了雷达设备，要把这里换成对应设备的序列号。

### 2. Faster-LIO（`src/faster-lio`）
- 基于 FastLIO2 改进的激光雷达-惯导里程计（LiDAR-Inertial Odometry）
- 输入：雷达点云 + IMU
- 输出：
  - `/Odometry`：雷达里程计位姿
  - `/cloud_registered`：配准后的点云（供 EGO-Planner 建图）
- 配置文件：`src/faster-lio/config/avia.yaml`

### 3. EKF 状态估计（`src/ekf_pose`）
- 融合 IMU（`/mavros/imu/data`）和 Faster-LIO 的里程计，输出更平滑的位姿
- 输出：`/ekf/ekf_odom`（全系统通用的位姿话题）
- 具备时间同步和异常值过滤（免疫 200ms 内断连和 30cm 以上跳变）
- 配置文件：`src/ekf_pose/launch/PX4_vio_drone.yaml`

### 4. MAVROS（`mavros`）
- 上位机（板载计算机）与 PX4 飞控之间的通信桥梁
- 通过串口连接 PX4 的 TELEM2 接口
- `shfiles/mavros.sh` 启动时同时配置关键消息的频率（IMU 333Hz、姿态 333Hz、位置 100Hz）

---

## 二、EGO-Planner 介绍

EGO-Planner 是核心轨迹规划器，位于 `src/planner/`。

### 内部模块

| 模块 | 说明 |
|------|------|
| `plan_manage/` | 顶层逻辑：重规划有限状态机（FSM）和规划管理器 |
| `plan_env/` | 环境表示，基于 `/cloud_registered` 实时维护占据栅格地图 |
| `path_searching/` | 动态 A* 搜索，寻找初始可行路径 |
| `traj_opt/` | 轨迹优化（MINCO，最小化控制代价 + 避障） |
| `traj_utils/` | 轨迹表示与数学工具 |

### 两种运行模式（`flight_type`）

在 `run_in_exp_interactive.launch` 中通过 `flight_type` 参数控制：

**`flight_type=1`（交互模式，默认）**
- 等待外部目标点输入，话题为 `/goal_with_id`（类型 `quadrotor_msgs/GoalSet`）
- 可以用 RViz 的"2D Nav Goal"工具手动给点
- **Mission Control 就是通过这个话题驱动 Planner 的**

**`flight_type=2`（预设航点模式）**
- 启动后自动按顺序飞 `run_in_exp_interactive.launch` 里配置的 `point0` ~ `pointN`
- 收到 `/traj_start_trigger` 话题后才开始执行
- 适合完全固定路线、无需上层状态机的场景

### 关键参数（`run_in_exp_interactive.launch`）

```xml
<arg name="max_vel" value="2.0" />       <!-- 最大速度 m/s -->
<arg name="max_acc" value="4.0" />       <!-- 最大加速度 m/s² -->
<arg name="planning_horizon" value="6.0" /> <!-- 规划视野，通常设为感知距离的1.5倍 -->
```

---

## 三、主要脚本说明

所有脚本位于 `shfiles/`，在**板载计算机**上执行。

### 完整系统启动：`run_gc.sh`

```bash
# 1. 打开串口权限
sudo chmod 777 /dev/tty*
# 2. 启动 MAVROS（与 PX4 通信）
roslaunch mavros px4.launch
# 3. 配置 MAVLink 消息频率（MAV_MSG_ID 105=高频IMU，31=姿态）
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0
# 4. 启动 Faster-LIO（雷达建图+里程计）
roslaunch faster_lio mapping_with_driver.launch
# 5. 启动 PX4 控制器
roslaunch px4ctrl run_ctrl.launch
# 6. 启动 EGO-Planner（交互模式，等待目标点）
roslaunch ego_planner run_in_exp_interactive.launch
```

**按顺序等待**，每步之间有 `sleep` 延迟，保证各节点正常初始化后再启动下一个。

### 其他脚本

| 脚本 | 作用 | 使用时机 |
|------|------|---------|
| `run_vis.sh` | 启动 RViz 可视化 | 在另一台电脑上单独运行，用于监控 |
| `takeoff.sh` | 启动 Mission Control 状态机 | 系统就绪后，触发任务开始 |
| `land.sh` | 直接发送降落指令 | 紧急或手动降落 |
| `shfiles/check_odom.sh` | 检查里程计话题 | 调试定位时用 |

### 标准操作流程

```
1. 上电，启动板载计算机
2. 执行 run_gc.sh  ← 等所有节点稳定（约15秒）
3. 遥控器切 OFFBOARD 模式
4. 执行 takeoff.sh ← 触发 Mission Control，无人机自动起飞+执行任务
```

---

## 四、Mission Control 状态机

核心文件：`src/mission_control/src/control_new.cpp`

### 状态流转

```
TAKING_OFF
    │  发送起飞指令到 /px4ctrl/takeoff_land
    ▼
INITIALIZING
    │  等待 5 秒，让无人机稳定在起飞高度
    ▼
TO_POINT_1
    │  发布 goal → /goal_with_id → EGO-Planner 规划轨迹
    │  到达（distance < tolerance）或超时（15s）后进入下一状态
    ▼
TO_POINT_2  →  TO_POINT_3  →  TO_POINT_4
    ▼
PRELAND
    │  发布停止指令 /stop_cmd，等待 6 秒稳定
    ▼
LANDING
       发送降落指令到 /px4ctrl/takeoff_land
```

### 关键逻辑细节

- **目标点发布**：每个状态只发布一次 `/goal_with_id`（`pub_flag` 控制），避免重复触发规划
- **到达判断**：计算当前位置（来自 `/ekf/ekf_odom`）与目标点的 **XY 平面距离** 小于 `tolerance`
- **超时保护**：每个航点最多等待 15 秒，超时自动进入下一状态（防止卡死）
- **飞行高度**：所有航点共用统一的 `fly_height`，z 轴高度固定

---

## 五、实现"起飞→多点导航→降落"需要改的地方

### 场景：修改航点位置和数量

#### 情况一：只需修改 4 个航点的坐标（最简单）

**只改一个文件**：`src/mission_control/config/mission_params_new.yaml`

```yaml
mission_control:
  fly_height: 1.2      # 飞行高度（米），根据实际环境调整
  tolerance: 0.15      # 到达判定半径（米），可按需添加这行

  # 航点1：起飞后的第一个目标
  point1_x: 2.0
  point1_y: 0.0

  # 航点2
  point2_x: 2.0
  point2_y: 3.0

  # 航点3
  point3_x: 0.0
  point3_y: 3.0

  # 航点4：最后一个点，之后降落
  point4_x: 0.0
  point4_y: 0.0
```

坐标系是**起飞点为原点的 ENU 坐标系**（x 向前，y 向左，z 向上），单位为米。

#### 情况二：需要改变航点数量

需要同时修改 `control_new.cpp` 和 yaml 配置文件。

**以改为 3 个航点为例**，修改 `src/mission_control/src/control_new.cpp`：

**第一步**：修改枚举（删除 `TO_POINT_4`）

```cpp
enum class State {
    TAKING_OFF,
    INITIALIZING,
    TO_POINT_1,
    TO_POINT_2,
    TO_POINT_3,   // 最后一个航点
    PRELAND,
    LANDING
};
```

**第二步**：修改构造函数中的 `loadWaypointParam` 调用（删掉 point4）

```cpp
loadWaypointParam("point1", waypoints[0]);
loadWaypointParam("point2", waypoints[1]);
loadWaypointParam("point3", waypoints[2]);
// 删掉 loadWaypointParam("point4", waypoints[3]);
```

**第三步**：修改 `run()` 里的状态转移（让 `TO_POINT_3` 直接跳 `PRELAND`）

```cpp
void run() {
    switch (state) {
        case State::TAKING_OFF:  takeOff(); break;
        case State::INITIALIZING: initialize(); break;
        case State::TO_POINT_1:  moveToPoint(0, State::TO_POINT_2); break;
        case State::TO_POINT_2:  moveToPoint(1, State::TO_POINT_3); break;
        case State::TO_POINT_3:  moveToPoint(2, State::PRELAND); break;  // ← 改这里
        case State::PRELAND:     preland(); break;
        case State::LANDING:     land(); break;
    }
}
```

对应地，yaml 里也只保留 `point1`~`point3`。

修改完后重新编译：

```bash
cd /path/to/gc_ws
catkin_make --pkg mission_control
source devel/setup.bash
```

### 参数调整建议

| 参数 | 文件 | 建议值 | 说明 |
|------|------|--------|------|
| `fly_height` | `mission_params_new.yaml` | 1.0~1.5m | 视障碍物高度决定 |
| `tolerance` | `mission_params_new.yaml` | 0.1~0.2m | 越小越精确但越难到达 |
| `takeoff_height` | `px4ctrl/config/ctrl_param_fpv.yaml` | 与 fly_height 一致 | px4ctrl 自动起飞的目标高度 |
| `max_vel` | `run_in_exp_interactive.launch` | 1.0~2.0m/s | 室内建议不超过 1.5 |

---

## 六、常见问题

**Q：执行 `takeoff.sh` 后无人机没有起飞**
- 检查 `px4ctrl` 节点是否已订阅 `/px4ctrl/takeoff_land`：`rostopic info /px4ctrl/takeoff_land`
- 确认遥控器已切到 OFFBOARD 模式

**Q：无人机起飞后悬停不走**
- 检查 EGO-Planner 是否收到目标点：`rostopic echo /goal_with_id`
- 检查 `/ekf/ekf_odom` 是否有数据：`rostopic hz /ekf/ekf_odom`

**Q：规划路径绕很大弯**
- 点云地图里有障碍物，检查 RViz 里的占据栅格是否正确
- 如果是第一次飞，`/cloud_registered` 需要几秒建图后才有完整地图

**Q：无人机到了航点附近但不继续**
- `tolerance` 设得太小，调大到 0.2m
- 或者是 15 秒超时机制会自动切换，等一下即可
