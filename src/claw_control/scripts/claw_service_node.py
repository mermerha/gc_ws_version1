#!/usr/bin/env python3
import time
import rospy
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from std_srvs.srv import Trigger, TriggerResponse
import serial
from uservo import UartServoManager

class ClawServiceNode:
    def __init__(self):
        rospy.init_node('claw_service_node')
        self.manager = None  # 防止在串口初始化完成前收到服务请求报错
        
        # 从参数服务器读取配置
        port     = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 115200)
        self.servo_id      = rospy.get_param('~servo_id', 0)
        self.angle_open    = rospy.get_param('~angle_open', 0.0)
        self.angle_close   = rospy.get_param('~angle_close', 25.0)
        self.move_interval = rospy.get_param('~move_interval', 2500)

        # [关键] 先注册 Service，再初始化硬件，避免时序问题
        rospy.Service('/claw/grab',    Trigger, self.handle_grab)
        rospy.Service('/claw/release', Trigger, self.handle_release)
        rospy.loginfo("[ClawService] Services registered. open=%.1f° close=%.1f°",
                      self.angle_open, self.angle_close)

        # 尝试清理可能占用该端口的僵尸进程 (自动解除占用)
        rospy.loginfo(f"[ClawService] Attempting to free port {port} from zombie processes...")
        os.system(f"fuser -k -9 {port} 2>/dev/null")
        time.sleep(0.5)

        # 初始化串口 + 舵机（与 claw_loop.py 相同逻辑，带重试）
        for init_attempt in range(3):
            try:
                rospy.loginfo(f"[ClawService] Init attempt {init_attempt+1}/3, opening port: {port}")
                self.uart = serial.Serial(port=port, baudrate=baudrate,
                                          parity=serial.PARITY_NONE,
                                          stopbits=1, bytesize=8, timeout=0)
                self.uart.reset_input_buffer()
                self.uart.reset_output_buffer()

                # USB 串口适配器需要时间稳定
                time.sleep(0.5)
                self.uart.reset_input_buffer()
                self.uart.reset_output_buffer()

                rospy.loginfo("[ClawService] Port opened. Scanning servos...")
                # 用 claw_loop.py 的方式：扫描全部舵机，比单独 ping 更稳定
                self.manager = UartServoManager(self.uart, is_debug=False)

                if self.servo_id not in self.manager.servos:
                    raise RuntimeError(f"Servo ID {self.servo_id} not found")

                rospy.loginfo(f"[ClawService] Servo ID {self.servo_id} found.")
                break
            except Exception as e:
                rospy.logwarn(f"[ClawService] Init attempt {init_attempt+1} failed: {e}")
                self.manager = None
                try:
                    self.uart.close()
                except Exception:
                    pass
                time.sleep(1)

        if self.manager is not None:
            # 启动时张开机械爪
            rospy.loginfo("[ClawService] Moving claw to initial open position...")
            self.manager.set_servo_angle(self.servo_id, self.angle_open,
                                         interval=self.move_interval)
            self.manager.wait()
            rospy.loginfo("[ClawService] Initialization complete. Ready.")
        else:
            rospy.logerr(f"[ClawService] All init attempts failed. Servo unavailable.")

    def handle_grab(self, req):
        """夹紧"""
        if self.manager is None:
            return TriggerResponse(success=False, message="Hardware not init")
        try:
            # 清空长期闲置可能积累的串口垃圾数据
            self.uart.reset_input_buffer()
            self.uart.reset_output_buffer()
            
            for attempt in range(3):
                rospy.loginfo(f"[ClawService] Grab attempt {attempt+1}/3")
                self.manager.set_servo_angle(self.servo_id, self.angle_close,
                                             interval=self.move_interval)
                self.manager.wait(timeout=3.0) # 加一个最大超时防止死等
                
                # 【核心改进】强制查询实际角度，进行闭环验证
                actual_angle = self.manager.query_servo_angle(self.servo_id)
                if actual_angle is not None and abs(actual_angle - self.angle_close) <= 5.0:
                    return TriggerResponse(success=True, message="GRABBED")
                
                rospy.logwarn(f"[ClawService] Grab attempt {attempt+1} failed. Target: {self.angle_close}, Actual: {actual_angle}. Retrying...")
                time.sleep(0.2)
                
            return TriggerResponse(success=False, message="Failed to reach target angle")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def handle_release(self, req):
        """松开"""
        if self.manager is None:
            return TriggerResponse(success=False, message="Hardware not init")
        try:
            # 清空长期闲置可能积累的串口垃圾数据
            self.uart.reset_input_buffer()
            self.uart.reset_output_buffer()
            
            for attempt in range(3):
                rospy.loginfo(f"[ClawService] Release attempt {attempt+1}/3")
                self.manager.set_servo_angle(self.servo_id, self.angle_open,
                                             interval=self.move_interval)
                self.manager.wait(timeout=3.0)
                
                # 【核心改进】强制查询实际角度，进行闭环验证
                actual_angle = self.manager.query_servo_angle(self.servo_id)
                if actual_angle is not None and abs(actual_angle - self.angle_open) <= 5.0:
                    return TriggerResponse(success=True, message="RELEASED")
                
                rospy.logwarn(f"[ClawService] Release attempt {attempt+1} failed. Target: {self.angle_open}, Actual: {actual_angle}. Retrying...")
                time.sleep(0.2)
                
            return TriggerResponse(success=False, message="Failed to reach target angle")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def shutdown(self):
        if self.manager is not None:
            try:
                self.manager.set_servo_angle(self.servo_id, self.angle_open,
                                             interval=self.move_interval)
                self.manager.wait()
                self.uart.close()
            except Exception:
                pass

    def spin(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

if __name__ == '__main__':
    node = ClawServiceNode()
    node.spin()
