#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import GoalSet, TakeoffLand
import numpy as np

class MockOdom:
    def __init__(self):
        rospy.init_node('mock_odom_node')
        
        # 发布虚拟里程计
        self.pub = rospy.Publisher('/ekf/ekf_odom', Odometry, queue_size=10)
        
        # 订阅目标点
        self.sub_goal = rospy.Subscriber('/goal_with_id', GoalSet, self.goal_cb)
        
        # 订阅起飞指令
        self.sub_takeoff = rospy.Subscriber('/px4ctrl/takeoff_land', TakeoffLand, self.takeoff_cb)
        
        self.curr_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos = np.array([0.0, 0.0, 0.0])
        self.speed = 0.15  # 模拟飞行速度 (m/frame)
        
        print("\n" + "="*40)
        print("   UAV FLIGHT SIMULATOR (GROUND TEST)   ")
        print("="*40 + "\n")
        rospy.loginfo("Mock Odom: Ready to simulate flight.")

    def goal_cb(self, msg):
        self.target_pos = np.array([msg.goal[0], msg.goal[1], msg.goal[2]])
        rospy.loginfo(">>> New Mission Goal Received: [%.2f, %.2f, %.2f]", 
                      self.target_pos[0], self.target_pos[1], self.target_pos[2])

    def takeoff_cb(self, msg):
        if msg.takeoff_land_cmd == 1:
            rospy.loginfo("!!! RECEIVED TAKEOFF COMMAND FROM MISSION CONTROL !!!")

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            direction = self.target_pos - self.curr_pos
            dist = np.linalg.norm(direction)
            
            if dist > 0.01:
                move = (direction / dist) * min(self.speed, dist)
                self.curr_pos += move
                
                # 每秒打印一次飞行进度
                rospy.loginfo_throttle(1.0, "[Flying] Current: [%.2f, %.2f, %.2f] -> Target: [%.2f, %.2f, %.2f] (Dist: %.2fm)",
                                       self.curr_pos[0], self.curr_pos[1], self.curr_pos[2],
                                       self.target_pos[0], self.target_pos[1], self.target_pos[2],
                                       dist)
            else:
                # 到达目标点后定时提醒
                rospy.loginfo_throttle(5.0, "[Hovering] At waypoint: [%.2f, %.2f, %.2f]",
                                       self.curr_pos[0], self.curr_pos[1], self.curr_pos[2])
            
            # 发布 Odometry
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "world"
            odom.pose.pose.position.x = self.curr_pos[0]
            odom.pose.pose.position.y = self.curr_pos[1]
            odom.pose.pose.position.z = self.curr_pos[2]
            odom.pose.pose.orientation.w = 1.0
            
            self.pub.publish(odom)
            rate.sleep()

if __name__ == '__main__':
    try:
        sim = MockOdom()
        sim.run()
    except rospy.ROSInterruptException:
        pass
