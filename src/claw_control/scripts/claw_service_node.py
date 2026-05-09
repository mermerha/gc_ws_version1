#!/usr/bin/env python3
import rospy
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
        self.angle_close   = rospy.get_param('~angle_close', 50.0)
        self.move_interval = rospy.get_param('~move_interval', 2500)

        # 初始化串口 + 舵机
        try:
            self.uart = serial.Serial(port=port, baudrate=baudrate,
                                      parity=serial.PARITY_NONE,
                                      stopbits=1, bytesize=8, timeout=1)
            self.manager = UartServoManager(self.uart, is_debug=False)

            # 启动时张开机械爪
            self.manager.set_servo_angle(self.servo_id, self.angle_open,
                                         interval=self.move_interval)
            self.manager.wait()
        except Exception as e:
            rospy.logerr(f"Failed to init servo on {port}: {e}")
            self.manager = None

        # 注册 Service
        rospy.Service('/claw/grab',    Trigger, self.handle_grab)
        rospy.Service('/claw/release', Trigger, self.handle_release)
        rospy.loginfo("[ClawService] Ready. open=%.1f° close=%.1f°",
                      self.angle_open, self.angle_close)

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
