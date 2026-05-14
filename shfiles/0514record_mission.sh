#!/bin/bash

# 确保 bag 文件夹存在
mkdir -p ~/gc_ws/bag

# 录制并保存到 ~/gc_ws/bag 目录下
rosbag record -O ~/gc_ws/bag/0514_mission_$(date +%Y%m%d_%H%M%S).bag \
    /ekf/ekf_odom \
    /cloud_registered \
    /drone_0_ego_planner_node/grid_map/occupancy \
    /drone_0_ego_planner_node/grid_map/occupancy_inflate \
    /drone_0_planning/trajectory \
    /drone_0_ego_planner_node/optimal_list \
    /mavros/setpoint_raw/attitude \
    /tf \
    /tf_static
