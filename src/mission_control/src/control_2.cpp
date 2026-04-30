#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <mavros_msgs/CommandBool.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/GoalSet.h>
#include <opencv_detect/Color_detection.h>

enum class State {
    TAKING_OFF,
    INITIALIZING,
    TO_POINT_1,
    TO_POINT_2,
    TO_POINT_3,
    TO_POINT_4,
    CATCH_BALL,
    UP,
    TO_POINT_5,
    TO_POINT_6,
    TO_POINT_7,
    TO_POINT_8,
    TO_POINT_9,
    TO_POINT_10,
    DROP_BALL,
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
        catch_pub = nh.advertise<std_msgs::Int32>("/catch", 10);
        drop_pub = nh.advertise<std_msgs::Int32>("/release", 10);
        color_client = nh.serviceClient<opencv_detect::Color_detection>("/detect_color", 10);

        // Load parameters
        nh.param<float>("/mission_control/tolerance", tolerance, 0.1);
        nh.param<float>("/mission_control/fly_height", fly_height, 0.0);
        nh.param<float>("/mission_control/fly_height_ball", fly_height_ball, 0.0);

        // Load waypoint parameters
        // Before catch
        loadWaypointParam("point1", waypoints[0]);
        loadWaypointParam("point2", waypoints[1]);
        loadWaypointParam("point3", waypoints[2]);
        loadWaypointParam("point4", waypoints[3]);
        // After catch, before drop
        loadWaypointParam("point5", waypoints[4]);
        loadWaypointParam("point6", waypoints[5]);
        loadWaypointParam("point7", waypoints[6]);
        // land points , RGB
        loadWaypointParam("land_point_r", land_points[0]);
        loadWaypointParam("land_point_g", land_points[1]);
        loadWaypointParam("land_point_b", land_points[2]);

        // Drop points, RGB
        loadWaypointParam("drop_point_r", drop_points[0]);
        loadWaypointParam("drop_point_g", drop_points[1]);
        loadWaypointParam("drop_point_b", drop_points[2]);
    }

    void run() {
        switch (state) {
            case State::TAKING_OFF: takeOff(); break;
            case State::INITIALIZING: initialize(); break;
            case State::TO_POINT_1: moveToPoint(0, State::TO_POINT_2); break;
            case State::TO_POINT_2: moveToPoint(1, State::TO_POINT_3); break;
            case State::TO_POINT_3: moveToPoint(2, State::TO_POINT_4); break;
            case State::TO_POINT_4: moveToPoint(3, State::CATCH_BALL); break;
            case State::CATCH_BALL: catchBall(); break;
            case State::UP: up(); break;
            case State::TO_POINT_5: moveToPoint(2, State::TO_POINT_6); break;
            case State::TO_POINT_6: moveToPoint(5, State::TO_POINT_7); break;
            case State::TO_POINT_7: moveToPoint(6, State::TO_POINT_8); break;
            case State::TO_POINT_8: moveToColorDropPoint(); break;
            case State::DROP_BALL: dropBall(); break;
            case State::TO_POINT_9: moveToColorPoint(); break;
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
    ros::Publisher catch_pub;
    ros::Publisher drop_pub;
    ros::Subscriber odometry_sub;
    ros::ServiceClient color_client;

    // State variables
    geometry_msgs::PoseStamped position_3d;
    quadrotor_msgs::GoalSet goal_with_id;
    std_msgs::Int32 cat, drop;
    int color_store = 0;
    int pub_flag = 1;
    ros::Time start_time;

    // Parameters
    float tolerance;
    float fly_height;
    float fly_height_ball;
    Point waypoints[15]; // point1 to point10
    Point drop_points[3];
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
        if (ros::Time::now() - start_time < ros::Duration(3.0)) return;
        pub_flag = 1;
        drop.data = 1;
        drop_pub.publish(drop);
        state = State::TO_POINT_1;
    }

    void moveToPoint(int point_idx, State next_state) {
        ROS_INFO("Flying to point %d...", point_idx + 1);
        if (goal_pub.getNumSubscribers() > 0) {
            if (pub_flag) {
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = waypoints[point_idx].x;
                goal_with_id.goal[1] = waypoints[point_idx].y;
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

    void catchBall() {
        ROS_INFO("Catching ball...");
        if (pub_flag) {
            goal_with_id.goal[2] = fly_height_ball;
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }
        if ((ros::Time::now() - start_time > ros::Duration(5.0)) || distance() < tolerance) {
            cat.data = 1;
            catch_pub.publish(cat);
            if (ros::Time::now() - start_time > ros::Duration(8.0)) {
                pub_flag = 1;
                start_time = ros::Time::now();
                state = State::UP;
            }
        }
    }

    void up() {
        ROS_INFO("Going up and recognizing color...");
        if (pub_flag) {
            goal_with_id.goal[2] = fly_height;
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }
        if ((ros::Time::now() - start_time > ros::Duration(5.0)) || distance() < tolerance) {
            opencv_detect::Color_detection srv;
            srv.request.detection_request = 1;
            if (color_client.call(srv)) {
                color_store = srv.response.color;
                ROS_INFO("Detected color: %d", color_store);
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::TO_POINT_5;
            }
        }
    }

    void moveToColorPoint() {
        ROS_INFO("Flying to land point...");
        if (goal_pub.getNumSubscribers() > 0) {
            if (pub_flag) {
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = land_points[color_store].x;
                goal_with_id.goal[1] = land_points[color_store].y;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0)) || distance() < tolerance) {
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::PRELAND;
            }
        }
    }

    void moveToColorDropPoint() {
        ROS_INFO("Flying to drop point...");
        if (goal_pub.getNumSubscribers() > 0) {
            if (pub_flag) {
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = drop_points[color_store].x;
                goal_with_id.goal[1] = drop_points[color_store].y;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0)) || distance() < tolerance) {
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::DROP_BALL;
            }
        }
    }

    void dropBall() {
        ROS_INFO("Dropping ball...");
        drop.data = 1;
        drop_pub.publish(drop);
        if (ros::Time::now() - start_time > ros::Duration(3.0)) {
            start_time = ros::Time::now();
            state = State::TO_POINT_9;
        }
    }

    void preland() {
        ROS_INFO("Prelanding...");
        if (distance() > tolerance) return;
        state = State::LANDING;
    }

    void land() {
        ROS_INFO("Landing...");
        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 2;
        if (takeoff_land_pub.getNumSubscribers() > 0 && pub_flag) {
            takeoff_land_pub.publish(takeoff_msg);
            pub_flag = 0;
        }
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