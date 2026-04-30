#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <mavros_msgs/CommandBool.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/GoalSet.h>
#include <std_msgs/Bool.h>

using namespace std;


enum class State {
    TAKING_OFF,
    INITIALIZING,
    TO_POINT_1,
    TO_POINT_2,
    TO_POINT_3,
    TO_POINT_4,
    PRELAND,
    LANDING
};

class MissionControl {
public:
    MissionControl() : state(State::TAKING_OFF) {
        // Initialize publishers and subscribers
        odometry_sub = nh.subscribe("/ekf/ekf_odom", 10, &MissionControl::odemetry_callback, this);
        goal_pub = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
        takeoff_land_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
        stop_cmd_pub = nh.advertise<std_msgs::Bool>("/stop_cmd", 10);

        // Load parameters
        nh.param<float>("/mission_control/tolerance", tolerance, 0.1);
        nh.param<float>("/mission_control/fly_height", fly_height, 0.0);
        //nh.param<float>("/mission_control/delta", delta, 0.0);

        // Load waypoint parameters
        loadWaypointParam("point1", waypoints[0]);
        loadWaypointParam("point2", waypoints[1]);
        loadWaypointParam("point3", waypoints[2]);
        loadWaypointParam("point4", waypoints[3]);

    }

    void run() {
        switch (state) {
            case State::TAKING_OFF: takeOff(); break;
            case State::INITIALIZING: initialize(); break;
            case State::TO_POINT_1: moveToPoint(0, State::TO_POINT_2); break;
            case State::TO_POINT_2: moveToPoint(1, State::TO_POINT_3); break;
            case State::TO_POINT_3: moveToPoint(2, State::TO_POINT_4); break;
            case State::TO_POINT_4: moveToPoint(3, State::PRELAND); break;
            case State::PRELAND: preland(); break;
            case State::LANDING: land(); break;
        }
    }
    private:
    struct Point {
        float x = 0.0;
        float y = 0.0;
    };

    State state;
    ros::NodeHandle nh;

    // ROS communication
    ros::Publisher takeoff_land_pub;
    ros::Publisher goal_pub;
    ros::Publisher stop_cmd_pub;
    ros::Subscriber odometry_sub;

    // State variables
    geometry_msgs::PoseStamped position_3d;
    quadrotor_msgs::GoalSet goal_with_id;
    int pub_flag = 1;
    ros::Time start_time;

    // Parameters
    float tolerance;
    float fly_height;
    float delta;
    Point waypoints[15]; // point1 to point10

    Point land_points[3];

    void loadWaypointParam(const std::string& name, Point& point) {
        nh.param<float>("/mission_control/" + name + "_x", point.x, 0.0);
        nh.param<float>("/mission_control/" + name + "_y", point.y, 0.0);
    }

    void takeOff() {
        ROS_INFO("Taking off...");
        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 1;
        if (takeoff_land_pub.getNumSubscribers() > 0) {
            takeoff_land_pub.publish(takeoff_msg);
            start_time = ros::Time::now();
            state = State::INITIALIZING;
        }
    }

    void initialize() {
        ROS_INFO("Initialing...");
        if (ros::Time::now() - start_time < ros::Duration(5.0)) return;
        // drop.data = 1;
        // drop_pub.publish(drop);
        pub_flag = 1;
        state = State::TO_POINT_1;
    }


    void moveToPoint(int point_idx, State next_state) {
        ROS_INFO("Flying to point %d...", point_idx + 1);
        if (goal_pub.getNumSubscribers() > 0) {
            if (pub_flag) {
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = waypoints[point_idx].x;
                goal_with_id.goal[1] = waypoints[point_idx].y;
                // if(point_idx == 3)
                //     goal_with_id.goal[2] = fly_height_ball + delta;
                //else goal_with_id.goal[2] = fly_height;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0)) || distance() < tolerance) {
                start_time = ros::Time::now();
                pub_flag = 1;
                state = next_state;
            }
        }
    }

    void preland() {
        ROS_INFO("Prelanding...");
    
        // 确保只发布一次
        if (pub_flag) {
            std_msgs::Bool stop_msg;
            stop_msg.data = true;
            stop_cmd_pub.publish(stop_msg);
        }
    
        if (ros::Time::now() - start_time > ros::Duration(6.0)) {
            start_time = ros::Time::now();
            state = State::LANDING;
        }

        if (distance() > tolerance) return;
        state = State::LANDING;
}

    void land() {
        ROS_INFO("Landing...");
        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 2;
        //if (takeoff_land_pub.getNumSubscribers() > 0) {
        takeoff_land_pub.publish(takeoff_msg);
        //}
    }

    double distance() {
        double dx = goal_with_id.goal[0] - position_3d.pose.position.x;
        double dy = goal_with_id.goal[1] - position_3d.pose.position.y;
        return sqrt(dx * dx + dy * dy);
    }

    void odemetry_callback(const nav_msgs::Odometry::ConstPtr &pMsg) {
        position_3d.pose = pMsg->pose.pose;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_control_node");
    MissionControl missionControl;
    ros::Rate rate(1);
    ros::param::set("/ready", 0);
    while (ros::ok()) {
        missionControl.run();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
