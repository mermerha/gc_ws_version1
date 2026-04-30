#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
颜色识别节点 - 持续发布模式
每秒检测一次当前画面中的主色，发布到 /detected_color 话题 (std_msgs/Int32)
同时在自己的终端持续打印识别结果
颜色编码: 0=未识别, 1=RED, 2=GREEN, 3=BLUE
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge


class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.current_image = None
        self.color_names = {0: "UNKNOWN", 1: "RED", 2: "GREEN", 3: "BLUE"}

        image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.min_pixel_threshold = rospy.get_param("~min_pixel_threshold", 500)
        self.roi_ratio = rospy.get_param("~roi_ratio", 0.6)

        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.color_pub = rospy.Publisher("/detected_color", Int32, queue_size=1)

        # 1Hz 定时器，每秒检测并发布一次
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        rospy.loginfo("[ColorDetector] Started. Publishing /detected_color at 1Hz")

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("[ColorDetector] cv_bridge error: %s", str(e))

    def detect_color(self):
        """检测当前帧的主色，返回 (color_code, r_count, g_count, b_count)"""
        if self.current_image is None:
            return 0, 0, 0, 0

        frame = self.current_image.copy()
        h, w = frame.shape[:2]

        # 裁剪中心区域
        r = self.roi_ratio
        y1, y2 = int(h * (1 - r) / 2), int(h * (1 + r) / 2)
        x1, x2 = int(w * (1 - r) / 2), int(w * (1 + r) / 2)
        roi = frame[y1:y2, x1:x2]

        roi = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # HSV 阈值
        mask_r = cv2.inRange(hsv, np.array([0, 80, 50]), np.array([10, 255, 255])) | \
                 cv2.inRange(hsv, np.array([160, 80, 50]), np.array([180, 255, 255]))
        mask_g = cv2.inRange(hsv, np.array([35, 80, 50]), np.array([85, 255, 255]))
        mask_b = cv2.inRange(hsv, np.array([100, 80, 50]), np.array([130, 255, 255]))

        rc = cv2.countNonZero(mask_r)
        gc = cv2.countNonZero(mask_g)
        bc = cv2.countNonZero(mask_b)

        counts = {1: rc, 2: gc, 3: bc}
        best = max(counts, key=counts.get)

        if counts[best] > self.min_pixel_threshold:
            return best, rc, gc, bc
        return 0, rc, gc, bc

    def timer_callback(self, event):
        color, rc, gc, bc = self.detect_color()
        self.color_pub.publish(Int32(data=color))
        rospy.loginfo("[Camera] >>> %s <<<  (R:%d  G:%d  B:%d)",
                      self.color_names[color], rc, gc, bc)


if __name__ == '__main__':
    rospy.init_node('color_detect_node')
    detector = ColorDetector()
    rospy.spin()
