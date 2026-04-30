#!/bin/bash
echo "===== 传感器检查 ====="

echo "1. 飞控:"
timeout 2 rostopic echo /mavros/state -n 1 2>/dev/null | grep "connected" || echo " 未连接"

echo "2. IMU:"
timeout 3 rostopic hz /mavros/imu/data --window=10 2>&1 | tail -1 || echo " 无数据"

echo "3. 雷达:"
timeout 3 rostopic hz /cloud_registered --window=5 2>&1 | tail -1 || echo " 无数据"

echo "4. 里程计:"
timeout 3 rostopic hz /Odometry --window=5 2>&1 | tail -1 || echo " 无数据"

echo "5. EKF:"
timeout 3 rostopic hz /ekf/ekf_odom --window=10 2>&1 | tail -1 || echo " 无数据"

echo "===== 检查完成 ====="
