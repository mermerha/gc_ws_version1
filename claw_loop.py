#!/usr/bin/env python3
# coding: utf-8
"""
机械爪循环测试脚本
功能：10° 停5秒 → 50° 停5秒 → 10° ... 无限循环
按 Ctrl+C 停止

运行方法：
    sudo chmod 666 /dev/ttyUSB0
    python3 claw_loop.py
"""

import sys, os, time, serial
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uservo import UartServoManager

# ============================================================
# 用户配置区
# ============================================================
SERVO_PORT_NAME = '/dev/ttyUSB0'
SERVO_BAUDRATE  = 115200
SERVO_ID        = 0

ANGLE_A         = 0.0   # 起始角度（度）
ANGLE_B         = 50.0    # 目标角度（度）
HOLD_SECONDS    = 5       # 每个位置停留时间（秒）
MOVE_INTERVAL   = 1000    # 单次运动时间（毫秒）
# ============================================================

print("=" * 50)
print("  FashionStar 机械爪 —— 循环测试")
print("  {}° <---> {}°，各停留{}秒".format(ANGLE_A, ANGLE_B, HOLD_SECONDS))
print("  按 Ctrl+C 停止")
print("=" * 50)

# 初始化串口
try:
    uart = serial.Serial(port=SERVO_PORT_NAME, baudrate=SERVO_BAUDRATE,
                         parity=serial.PARITY_NONE, stopbits=1,
                         bytesize=8, timeout=0)
    print("[OK] 串口打开成功: {}".format(SERVO_PORT_NAME))
except serial.SerialException as e:
    print("[ERROR] 串口打开失败: {}".format(e))
    sys.exit(1)

# 初始化舵机管理器
print("[扫描] 正在检测舵机...")
uservo = UartServoManager(uart, is_debug=False)

if SERVO_ID not in uservo.servos:
    print("[ERROR] 舵机 ID={} 未响应".format(SERVO_ID))
    uart.close()
    sys.exit(1)

print("[OK] 检测到舵机 ID={}".format(SERVO_ID))

# 先移动到初始角度
print("\n[初始化] 移动到起始角度 {}°...".format(ANGLE_A))
uservo.set_servo_angle(SERVO_ID, ANGLE_A, interval=MOVE_INTERVAL)
uservo.wait()
print("         -> 当前角度: {:.1f}°".format(uservo.query_servo_angle(SERVO_ID)))

# 循环
loop_count = 0
try:
    while True:
        loop_count += 1
        print("\n--- 第 {} 轮 ---".format(loop_count))

        # 停留在 ANGLE_A
        print("[停留] {}°，等待 {}秒...".format(ANGLE_A, HOLD_SECONDS))
        time.sleep(HOLD_SECONDS)

        # 转到 ANGLE_B
        print("[移动] → {}°".format(ANGLE_B))
        uservo.set_servo_angle(SERVO_ID, ANGLE_B, interval=MOVE_INTERVAL)
        uservo.wait()
        print("        -> 当前角度: {:.1f}°".format(uservo.query_servo_angle(SERVO_ID)))

        # 停留在 ANGLE_B
        print("[停留] {}°，等待 {}秒...".format(ANGLE_B, HOLD_SECONDS))
        time.sleep(HOLD_SECONDS)

        # 转回 ANGLE_A
        print("[移动] → {}°".format(ANGLE_A))
        uservo.set_servo_angle(SERVO_ID, ANGLE_A, interval=MOVE_INTERVAL)
        uservo.wait()
        print("        -> 当前角度: {:.1f}°".format(uservo.query_servo_angle(SERVO_ID)))

except KeyboardInterrupt:
    print("\n\n[停止] 收到 Ctrl+C，退出循环。")

uart.close()
print("[完成] 程序结束。")