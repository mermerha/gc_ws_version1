#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Color Detection Node for Aerial Robot Competition.
Subscribes to camera image, provides /detect_color service.
Returns: 0=unknown, 1=red, 2=green, 3=blue
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from opencv_detect.srv import Color_detection, Color_detectionResponse


class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.current_image = None

        # Configurable image topic
        image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")

        # Min pixel count to consider a valid detection
        self.min_pixel_threshold = rospy.get_param("~min_pixel_threshold", 500)

        # Number of consecutive detections for majority vote
        self.vote_count = rospy.get_param("~vote_count", 5)

        # ROI ratio - center crop ratio for focusing on the target
        self.roi_ratio = rospy.get_param("~roi_ratio", 0.6)

        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.service = rospy.Service("/detect_color", Color_detection, self.handle_detect)

        self.color_names = {0: "UNKNOWN", 1: "RED", 2: "GREEN", 3: "BLUE"}

        rospy.loginfo("[ColorDetector] Initialized. Listening on: %s", image_topic)

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("[ColorDetector] cv_bridge error: %s", str(e))

    def detect_single_frame(self):
        """Detect dominant color in the current frame. Returns 0/1/2/3."""
        if self.current_image is None:
            return 0

        frame = self.current_image.copy()
        h, w = frame.shape[:2]

        # Crop center ROI
        r = self.roi_ratio
        y1, y2 = int(h * (1 - r) / 2), int(h * (1 + r) / 2)
        x1, x2 = int(w * (1 - r) / 2), int(w * (1 + r) / 2)
        roi = frame[y1:y2, x1:x2]

        # Gaussian blur to reduce noise
        roi = cv2.GaussianBlur(roi, (5, 5), 0)

        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- HSV ranges ---
        # Red wraps around H=0/180
        lower_red1 = np.array([0, 80, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 80, 50])
        upper_red2 = np.array([180, 255, 255])

        lower_green = np.array([35, 80, 50])
        upper_green = np.array([85, 255, 255])

        lower_blue = np.array([100, 80, 50])
        upper_blue = np.array([130, 255, 255])

        # Create masks
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | \
                   cv2.inRange(hsv, lower_red2, upper_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Morphological opening to remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        # Count pixels
        red_count = cv2.countNonZero(mask_red)
        green_count = cv2.countNonZero(mask_green)
        blue_count = cv2.countNonZero(mask_blue)

        counts = {1: red_count, 2: green_count, 3: blue_count}
        max_color = max(counts, key=counts.get)

        if counts[max_color] > self.min_pixel_threshold:
            return max_color
        return 0

    def handle_detect(self, req):
        """Service handler: perform majority vote over multiple frames."""
        resp = Color_detectionResponse()

        if self.current_image is None:
            rospy.logwarn("[ColorDetector] No image received yet!")
            resp.color = 0
            return resp

        # Majority vote
        votes = []
        rate = rospy.Rate(10)
        for i in range(self.vote_count):
            c = self.detect_single_frame()
            votes.append(c)
            rate.sleep()

        # Count votes (exclude 0=unknown)
        vote_map = {1: 0, 2: 0, 3: 0}
        for v in votes:
            if v in vote_map:
                vote_map[v] += 1

        best = max(vote_map, key=vote_map.get)
        if vote_map[best] > 0:
            resp.color = best
        else:
            resp.color = 0

        rospy.loginfo("============================================")
        rospy.loginfo("  COLOR DETECTION RESULT: %s", self.color_names[resp.color])
        rospy.loginfo("  Votes: R=%d  G=%d  B=%d  (from %d frames)",
                       vote_map[1], vote_map[2], vote_map[3], self.vote_count)
        rospy.loginfo("============================================")

        return resp


if __name__ == '__main__':
    rospy.init_node('color_detect_node')
    detector = ColorDetector()
    rospy.loginfo("[ColorDetector] Service /detect_color ready.")
    rospy.spin()
