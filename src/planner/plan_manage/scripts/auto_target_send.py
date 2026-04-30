#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import GoalSet
from nav_msgs.msg import Odometry
from math import sqrt

pred_goal = [[3.4, 2.5, 1.2], [7.5, 2.6, 1.2], [7.45, -0.0, 1.2], [0.0, 0.0, 1.2]]

num = 4


class GoalSender:
    def __init__(self):
        rospy.init_node("goal_sender", anonymous=True)

        self.cnt = 0
        self.target_pose = GoalSet()
        self.target_pose.goal[0] = pred_goal[self.cnt][0]
        self.target_pose.goal[1] = pred_goal[self.cnt][1]
        self.target_pose.goal[2] = pred_goal[self.cnt][2]

        self.current_position = None
        rospy.Subscriber("/ekf/ekf_odom", Odometry, self.odom_callback)
        rospy.Subscriber("/traj_start_trigger", PoseStamped, self.trigger_callback)

        self.goal_pub = rospy.Publisher(
            "/goal_with_id", GoalSet, queue_size=10
        )

        self.threshold = 1.0

        self.rate = rospy.Rate(100)  # 100 Hz
        self.start_trigger = False

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def trigger_callback(self, msg):
        self.start_trigger = True
        goal_sender.send_goal()

    def check_distance(self, target, current):
        dx = target.goal[0] - current.x
        dy = target.goal[1] - current.y
        return sqrt(dx**2 + dy**2)

    def send_goal(self):
        rospy.loginfo("Sending target goal: %s", self.target_pose)
        self.goal_pub.publish(self.target_pose)

    def run(self):
        while not rospy.is_shutdown():
            if self.start_trigger and self.current_position and self.cnt < num - 1:
                distance = self.check_distance(self.target_pose, self.current_position)
                if distance < self.threshold:
                    rospy.loginfo("Goal reached! Sending new goal...")
                    self.cnt += 1
                    print(self.cnt)
                    self.target_pose.goal[0] = pred_goal[self.cnt][0]
                    self.target_pose.goal[1]= pred_goal[self.cnt][1]
                    self.target_pose.goal[2]= pred_goal[self.cnt][2]
                    self.send_goal()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        goal_sender = GoalSender()
        goal_sender.run()
    except rospy.ROSInterruptException:
        pass
