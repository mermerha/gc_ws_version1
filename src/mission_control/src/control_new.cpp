#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/CommandBool.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/GoalSet.h>
#include <opencv_detect/Color_detection.h>

using namespace std;

enum class State {
    TAKING_OFF,
    INITIALIZING,
    TO_POINT_1,
    TO_POINT_2,
    HOVER_FOR_COLOR,    // hover & lower height at A for color detection
    IDENTIFY_COLOR,     // call color service, print result (Task 7)
    TO_POINT_3,
    TO_POINT_4,
    TO_LAND_POINT,      // fly to the correct R/G/B landing pad (Task 8)
    PRELAND,
    LANDING
};

class MissionControl {
public:
    MissionControl() : state(State::TAKING_OFF), color_store(0), pub_flag(1) {
        // Publishers & Subscribers
        odometry_sub = nh.subscribe("/ekf/ekf_odom", 10, &MissionControl::odemetry_callback, this);
        goal_pub = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
        takeoff_land_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
        stop_cmd_pub = nh.advertise<std_msgs::Bool>("/stop_cmd", 10);

        // Color detection service client
        color_client = nh.serviceClient<opencv_detect::Color_detection>("/detect_color");

        // Load parameters
        nh.param<float>("/mission_control/tolerance", tolerance, 0.3);
        nh.param<float>("/mission_control/fly_height", fly_height, 1.2);
        nh.param<float>("/mission_control/fly_height_color", fly_height_color, 0.8);
        nh.param<int>("/mission_control/color_detect_attempts", color_detect_attempts, 3);

        // Load navigation waypoints
        loadWaypointParam("point1", waypoints[0]);
        loadWaypointParam("point2", waypoints[1]);
        loadWaypointParam("point3", waypoints[2]);
        loadWaypointParam("point4", waypoints[3]);

        // Load R/G/B landing coordinates at B area
        // Index: 1=Red, 2=Green, 3=Blue (matches color_store values)
        nh.param<float>("/mission_control/land_point_r_x", land_points[1].x, 0.0);
        nh.param<float>("/mission_control/land_point_r_y", land_points[1].y, 0.0);
        nh.param<float>("/mission_control/land_point_g_x", land_points[2].x, 0.0);
        nh.param<float>("/mission_control/land_point_g_y", land_points[2].y, 0.0);
        nh.param<float>("/mission_control/land_point_b_x", land_points[3].x, 0.0);
        nh.param<float>("/mission_control/land_point_b_y", land_points[3].y, 0.0);

        ROS_INFO("[MissionControl] Initialized. Waypoints and landing points loaded.");
    }

    void run() {
        switch (state) {
            case State::TAKING_OFF:     takeOff(); break;
            case State::INITIALIZING:   initialize(); break;
            case State::TO_POINT_1:     moveToPoint(0, State::TO_POINT_2); break;
            case State::TO_POINT_2:     moveToPoint(1, State::HOVER_FOR_COLOR); break;
            case State::HOVER_FOR_COLOR: hoverForColor(); break;
            case State::IDENTIFY_COLOR: identifyColor(); break;
            case State::TO_POINT_3:     moveToPoint(2, State::TO_POINT_4); break;
            case State::TO_POINT_4:     moveToPoint(3, State::TO_LAND_POINT); break;
            case State::TO_LAND_POINT:  moveToLandPoint(); break;
            case State::PRELAND:        preland(); break;
            case State::LANDING:        land(); break;
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
    ros::ServiceClient color_client;

    // State variables
    geometry_msgs::PoseStamped position_3d;
    quadrotor_msgs::GoalSet goal_with_id;
    int pub_flag;
    ros::Time start_time;

    // Color detection
    int color_store;            // 0=unknown, 1=red, 2=green, 3=blue
    int color_detect_attempts;
    string color_names[4] = {"UNKNOWN", "RED", "GREEN", "BLUE"};

    // Parameters
    float tolerance;
    float fly_height;
    float fly_height_color;
    Point waypoints[15];
    Point land_points[4];       // index 1=R, 2=G, 3=B

    void loadWaypointParam(const std::string& name, Point& point) {
        nh.param<float>("/mission_control/" + name + "_x", point.x, 0.0);
        nh.param<float>("/mission_control/" + name + "_y", point.y, 0.0);
    }

    // ============ State Handlers ============

    void takeOff() {
        ROS_INFO("[State] TAKING_OFF");
        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 1;
        if (takeoff_land_pub.getNumSubscribers() > 0) {
            takeoff_land_pub.publish(takeoff_msg);
            start_time = ros::Time::now();
            state = State::INITIALIZING;
        }
    }

    void initialize() {
        ROS_INFO("[State] INITIALIZING");
        if (ros::Time::now() - start_time < ros::Duration(5.0)) return;
        pub_flag = 1;
        start_time = ros::Time::now();
        state = State::TO_POINT_1;
    }

    void moveToPoint(int point_idx, State next_state) {
        ROS_INFO("[State] Flying to point %d (%.2f, %.2f)...",
                 point_idx + 1, waypoints[point_idx].x, waypoints[point_idx].y);
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

    /**
     * Hover at A point and optionally lower height for better camera view.
     * Then transition to IDENTIFY_COLOR.
     */
    void hoverForColor() {
        ROS_INFO("[State] HOVER_FOR_COLOR - stabilizing at A point...");
        if (pub_flag) {
            // Optionally lower to fly_height_color for better detection
            goal_with_id.goal[2] = fly_height_color;
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }
        // Wait for drone to stabilize at the lower height
        if (ros::Time::now() - start_time > ros::Duration(3.0)) {
            start_time = ros::Time::now();
            pub_flag = 1;
            state = State::IDENTIFY_COLOR;
        }
    }

    /**
     * Task 7 (15 points): Call color detection service, print result to terminal.
     * Tries multiple times for robustness.
     */
    void identifyColor() {
        ROS_INFO("[State] IDENTIFY_COLOR - detecting target color...");

        opencv_detect::Color_detection srv;
        srv.request.detection_request = 1;

        int best_color = 0;
        int votes[4] = {0, 0, 0, 0};

        // Call service multiple times for robustness
        for (int i = 0; i < color_detect_attempts; i++) {
            if (color_client.call(srv)) {
                int c = srv.response.color;
                if (c >= 1 && c <= 3) {
                    votes[c]++;
                }
            } else {
                ROS_WARN("[IDENTIFY_COLOR] Service call failed (attempt %d/%d)",
                         i + 1, color_detect_attempts);
            }
            ros::Duration(0.5).sleep();
        }

        // Pick majority
        int max_votes = 0;
        for (int i = 1; i <= 3; i++) {
            if (votes[i] > max_votes) {
                max_votes = votes[i];
                best_color = i;
            }
        }

        color_store = best_color;

        // ====== Task 7 scoring: print color to terminal ======
        ROS_INFO("======================================================");
        ROS_INFO("  [TASK 7] IDENTIFIED COLOR AT A: >>> %s <<<",
                 color_names[color_store].c_str());
        ROS_INFO("  Votes: RED=%d  GREEN=%d  BLUE=%d", votes[1], votes[2], votes[3]);
        ROS_INFO("======================================================");

        if (color_store == 0) {
            ROS_WARN("[IDENTIFY_COLOR] Could not detect color! Defaulting to RED.");
            color_store = 1;
        }

        // Restore normal flight height and proceed to B
        if (pub_flag) {
            goal_with_id.goal[2] = fly_height;
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }

        start_time = ros::Time::now();
        pub_flag = 1;
        state = State::TO_POINT_3;
    }

    /**
     * Task 8 (15 points): Fly to the landing pad matching the detected color.
     */
    void moveToLandPoint() {
        ROS_INFO("[State] TO_LAND_POINT - flying to %s landing pad (%.2f, %.2f)...",
                 color_names[color_store].c_str(),
                 land_points[color_store].x, land_points[color_store].y);

        if (goal_pub.getNumSubscribers() > 0) {
            if (pub_flag) {
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = land_points[color_store].x;
                goal_with_id.goal[1] = land_points[color_store].y;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;

                ROS_INFO("======================================================");
                ROS_INFO("  [TASK 8] LANDING ON %s PAD", color_names[color_store].c_str());
                ROS_INFO("======================================================");
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0)) || distance() < tolerance) {
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::PRELAND;
            }
        }
    }

    void preland() {
        ROS_INFO("[State] PRELAND");
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
        ROS_INFO("[State] LANDING");
        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 2;
        takeoff_land_pub.publish(takeoff_msg);
    }

    // ============ Utilities ============

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
