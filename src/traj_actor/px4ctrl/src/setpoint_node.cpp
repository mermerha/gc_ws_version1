#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/RCIn.h>
#include <vector>
#include <cmath>
#include <XmlRpcValue.h>

// 航点结构体
struct Waypoint {
    int index;
    double x;
    double y;
    double z;
};

mavros_msgs::State current_state;
geometry_msgs::PoseStamped posefly;
bool take_off_has_pub = false;
uint channel8;
ros::Time begin_time;
ros::Time waypoint_start_time;
std::vector<Waypoint> waypoints;
bool landing_command_sent = false;
int count = 0;
int current_waypoint_index = -1;
const double POSITION_TOLERANCE = 0.2;  // 位置容差（米）
const double WAYPOINT_TIMEOUT = 5.0;    // 航点超时时间（秒）

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    posefly = *msg;
}

void rcCallback(const mavros_msgs::RCIn::ConstPtr &msg) {
    mavros_msgs::RCIn rc_in = *msg;
    channel8 = (rc_in.channels)[7];
}

// 从参数服务器读取航点
void loadWaypoints(ros::NodeHandle& nh) {
    XmlRpc::XmlRpcValue waypoints_param;
    if (nh.getParam("waypoints", waypoints_param)) {
        ROS_ASSERT(waypoints_param.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
        for (int i = 0; i < waypoints_param.size(); ++i) {
            XmlRpc::XmlRpcValue wp_param = waypoints_param[i];
            ROS_ASSERT(wp_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);
            
            Waypoint wp;
            wp.index = static_cast<int>(wp_param["index"]);
            wp.x = static_cast<double>(wp_param["x"]);
            wp.y = static_cast<double>(wp_param["y"]);
            wp.z = static_cast<double>(wp_param["z"]);
            
            waypoints.push_back(wp);
            ROS_INFO("Loaded waypoint %d: (%.2f, %.2f, %.2f)", 
                     wp.index, wp.x, wp.y, wp.z);
        }
        ROS_INFO("Total %zu waypoints loaded", waypoints.size());
    } 
}

// 检查是否到达当前航点
bool checkWaypointReached() {
    if (current_waypoint_index < 0 || current_waypoint_index >= waypoints.size()) 
        return false;
    
    const Waypoint& target = waypoints[current_waypoint_index];
    double dx = target.x - posefly.pose.position.x;
    double dy = target.y - posefly.pose.position.y;
    double dz = target.z - posefly.pose.position.z;
    
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    return distance < POSITION_TOLERANCE;
}

// 发布航点命令
void publishWaypointCmd(ros::Publisher& pub) {
    if (current_waypoint_index < 0 || current_waypoint_index >= waypoints.size()) 
        return;
    
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    
    const Waypoint& target = waypoints[current_waypoint_index];
    cmd.position.x = target.x;
    cmd.position.y = target.y;
    cmd.position.z = target.z;
    
    pub.publish(cmd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");
    
    // 从参数服务器加载航点
    loadWaypoints(nh);
    
    // 发布者
    ros::Publisher takeoff_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
    ros::Publisher position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
    
    // 订阅者
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, rcCallback);
    
    ros::Rate rate(50.0);  // 50Hz控制频率
    
    quadrotor_msgs::TakeoffLand takeoff_land;
    begin_time = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();
        
        // 处理起飞指令
        if (channel8 > 1900 && count < 30) {
            takeoff_land.takeoff_land_cmd = 1; // 起飞指令
            takeoff_pub.publish(takeoff_land);
            begin_time = ros::Time::now();
            count++;
            ROS_INFO("Takeoff command sent");
            
            // 起飞后设置第一个航点
            current_waypoint_index = 0;
            waypoint_start_time = ros::Time::now();
        }
        
        // 如果已经起飞，执行航点飞行
        if (current_waypoint_index >= 0 && current_waypoint_index < waypoints.size()) {
            // 发布当前航点命令
            publishWaypointCmd(position_cmd_pub);
            
            // 检查是否到达当前航点
            if (checkWaypointReached()) {
                ROS_INFO("Reached waypoint %d: (%.2f, %.2f, %.2f)", 
                         waypoints[current_waypoint_index].index,
                         waypoints[current_waypoint_index].x,
                         waypoints[current_waypoint_index].y,
                         waypoints[current_waypoint_index].z);
                
                // 切换到下一个航点
                current_waypoint_index++;
                waypoint_start_time = ros::Time::now();
                
                if (current_waypoint_index >= waypoints.size()) {
                    ROS_INFO("All waypoints completed");
                }
            }
            // 检查航点超时
            else if ((ros::Time::now() - waypoint_start_time).toSec() > WAYPOINT_TIMEOUT) {
                ROS_WARN("Waypoint %d timeout after %.1f seconds. Landing...", 
                         waypoints[current_waypoint_index].index, WAYPOINT_TIMEOUT);
                
                takeoff_land.takeoff_land_cmd = 2; // 降落指令
                takeoff_pub.publish(takeoff_land);
                landing_command_sent = true;
                break; // 退出循环
            }
        }
        
        // 处理降落指令（遥控器中位或完成所有航点）
        if (!landing_command_sent && 
            ((current_waypoint_index >= waypoints.size()) || 
             (channel8 > 1000 && channel8 < 1100))) 
        {
            takeoff_land.takeoff_land_cmd = 2; // 降落指令
            takeoff_pub.publish(takeoff_land);
            landing_command_sent = true;
            ROS_INFO("Landing command sent");
        }
        
        rate.sleep();
    }

    return 0;
}