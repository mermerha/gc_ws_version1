#!/usr/bin/env python3
import rospy
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from std_srvs.srv import Trigger, TriggerResponse
import serial
from uservo import UartServoManager

class ClawServiceNode:
    def __init__(self):
        rospy.init_node('claw_service_node')

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

        # 初始化串口 + 舵机（异步进行，不阻塞服务响应）
        try:
            rospy.loginfo(f"[ClawService] Attempting to open port: {port}")
            self.uart = serial.Serial(port=port, baudrate=baudrate,
                                      parity=serial.PARITY_NONE,
                                      stopbits=1, bytesize=8, timeout=0)
            self.uart.reset_input_buffer()
            self.uart.reset_output_buffer()

            rospy.loginfo("[ClawService] Port opened successfully. Initializing Manager...")
            self.manager = UartServoManager(self.uart, is_scan_servo=False, is_debug=False)

            rospy.loginfo(f"[ClawService] Pinging servo ID {self.servo_id}...")
            if not self.manager.ping(self.servo_id):
                rospy.logwarn(f"[ClawService] Servo ID {self.servo_id} not responding, but continuing...")

            # 启动时张开机械爪
            rospy.loginfo("[ClawService] Moving claw to initial open position...")
            self.manager.set_servo_angle(self.servo_id, self.angle_open,
                                         interval=self.move_interval)
            self.manager.wait()
            rospy.loginfo("[ClawService] Initialization sequence complete. Ready.")
        except Exception as e:
            rospy.logerr(f"Failed to init servo on {port}: {e}")
            self.manager = None

    def handle_grab(self, req):
        """夹紧"""
        if self.manager is None:
            return TriggerResponse(success=False, message="Hardware not init")
        try:
            self.manager.set_servo_angle(self.servo_id, self.angle_close,
                                         interval=self.move_interval)
            self.manager.wait()
            return TriggerResponse(success=True, message="GRABBED")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def handle_release(self, req):
        """松开"""
        if self.manager is None:
            return TriggerResponse(success=False, message="Hardware not init")
        try:
            self.manager.set_servo_angle(self.servo_id, self.angle_open,
                                         interval=self.move_interval)
            self.manager.wait()
            return TriggerResponse(success=True, message="RELEASED")
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
