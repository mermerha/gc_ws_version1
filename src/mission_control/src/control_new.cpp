#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/GoalSet.h>

using namespace std;

enum class State {
    TAKING_OFF,
    INITIALIZING,
    NAVIGATING,          // 按顺序飞航点
    IDENTIFY_COLOR,      // 在 A 点读取颜色
    TO_COLOR_ENDPOINT,   // 飞向颜色对应的终点
    VERIFY_COLOR,        // 在终点验证颜色
    PRELAND,
    LANDING
};

class MissionControl {
public:
    MissionControl() : state(State::TAKING_OFF), color_store(0),
                       latest_color(0), current_point_idx(0), pub_flag(1) {
        odometry_sub = nh.subscribe("/ekf/ekf_odom", 10, &MissionControl::odom_callback, this);
        color_sub = nh.subscribe("/detected_color", 1, &MissionControl::color_callback, this);
        goal_pub = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
        takeoff_land_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
        stop_cmd_pub = nh.advertise<std_msgs::Bool>("/stop_cmd", 10);

        // 通用参数
        nh.param<float>("/mission_control/tolerance", tolerance, 0.3);
        nh.param<float>("/mission_control/fly_height", fly_height, 1.2);
        nh.param<int>("/mission_control/num_points", num_points, 5);

        // A 点和 B 点的索引 (1-indexed，和 yaml 里 point1, point2... 对应)
        nh.param<int>("/mission_control/a_point_index", a_point_index, 2);
        nh.param<int>("/mission_control/b_point_index", b_point_index, 5);

        // 读取所有航点
        for (int i = 0; i < num_points && i < 15; ++i) {
            string name = "point" + to_string(i + 1);
            nh.param<float>("/mission_control/" + name + "_x", waypoints[i].x, 0.0);
            nh.param<float>("/mission_control/" + name + "_y", waypoints[i].y, 0.0);
        }

        // 三种颜色的终点坐标 (索引: 1=Red, 2=Green, 3=Blue)
        nh.param<float>("/mission_control/color_end_r_x", color_endpoints[1].x, 0.0);
        nh.param<float>("/mission_control/color_end_r_y", color_endpoints[1].y, 0.0);
        nh.param<float>("/mission_control/color_end_g_x", color_endpoints[2].x, 0.0);
        nh.param<float>("/mission_control/color_end_g_y", color_endpoints[2].y, 0.0);
        nh.param<float>("/mission_control/color_end_b_x", color_endpoints[3].x, 0.0);
        nh.param<float>("/mission_control/color_end_b_y", color_endpoints[3].y, 0.0);

        ROS_INFO("[MissionControl] num_points=%d, A=point%d, B=point%d", num_points, a_point_index, b_point_index);
    }

    void run() {
        switch (state) {
            case State::TAKING_OFF:       takeOff(); break;
            case State::INITIALIZING:     initialize(); break;
            case State::NAVIGATING:       navigate(); break;
            case State::IDENTIFY_COLOR:   identifyColor(); break;
            case State::TO_COLOR_ENDPOINT:toColorEndpoint(); break;
            case State::VERIFY_COLOR:     verifyColor(); break;
            case State::PRELAND:          preland(); break;
            case State::LANDING:          land(); break;
        }
    }

private:
    struct Point { float x = 0.0; float y = 0.0; };

    State state;
    ros::NodeHandle nh;
    ros::Publisher takeoff_land_pub, goal_pub, stop_cmd_pub;
    ros::Subscriber odometry_sub, color_sub;

    geometry_msgs::PoseStamped position_3d;
    quadrotor_msgs::GoalSet goal_with_id;
    ros::Time start_time;
    int pub_flag;

    // 航点
    Point waypoints[15];
    int num_points;
    int current_point_idx;   // 0-indexed

    // A/B 索引 (1-indexed)
    int a_point_index;
    int b_point_index;

    // 颜色
    int latest_color;        // 来自话题的实时颜色
    int color_store;         // 在 A 点记住的颜色
    int color_votes[4];      // 颜色投票计数 [0~3]
    Point color_endpoints[4]; // 1=R终点, 2=G终点, 3=B终点
    string color_names[4] = {"UNKNOWN", "RED", "GREEN", "BLUE"};

    float tolerance;
    float fly_height;

    // ============ 回调 ============
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
        position_3d.pose = msg->pose.pose;
    }

    void color_callback(const std_msgs::Int32::ConstPtr &msg) {
        latest_color = msg->data;
    }

    // ============ 状态处理 ============
    void takeOff() {
        quadrotor_msgs::TakeoffLand msg;
        msg.takeoff_land_cmd = 1;
        if (takeoff_land_pub.getNumSubscribers() > 0) {
            takeoff_land_pub.publish(msg);
            start_time = ros::Time::now();
            state = State::INITIALIZING;
            ROS_INFO("[State] TAKING_OFF");
        }
    }

    void initialize() {
        if (ros::Time::now() - start_time < ros::Duration(5.0)) return;
        current_point_idx = 0;
        pub_flag = 1;
        start_time = ros::Time::now();
        state = State::NAVIGATING;
        ROS_INFO("[State] INITIALIZING done, starting navigation");
    }

    /**
     * 通用导航状态：按顺序飞航点
     * 到达 A 点 → 跳转 IDENTIFY_COLOR
     * 到达 B 点 → 跳转 TO_COLOR_ENDPOINT
     * 飞完所有点 → 跳转 PRELAND
     */
    void navigate() {
        if (current_point_idx >= num_points) {
            state = State::PRELAND;
            start_time = ros::Time::now();
            return;
        }

        // 发布当前目标航点
        if (pub_flag) {
            ROS_INFO("[NAVIGATING] Flying to point %d (%.2f, %.2f)",
                     current_point_idx + 1,
                     waypoints[current_point_idx].x,
                     waypoints[current_point_idx].y);
            goal_with_id.drone_id = 0;
            goal_with_id.goal[0] = waypoints[current_point_idx].x;
            goal_with_id.goal[1] = waypoints[current_point_idx].y;
            goal_with_id.goal[2] = fly_height;
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }

        // 到达判定（加超时保护：15秒）
        bool arrived = (distance() < tolerance);
        bool timeout = (ros::Time::now() - start_time > ros::Duration(15.0));

        if (arrived || timeout) {
            int arrived_point = current_point_idx + 1;
            if (timeout && !arrived) {
                ROS_WARN("[NAVIGATING] Timeout at point %d, forcing advance", arrived_point);
            } else {
                ROS_INFO("[NAVIGATING] Arrived at point %d", arrived_point);
            }

            if (arrived_point == a_point_index) {
                state = State::IDENTIFY_COLOR;
                color_votes[0] = color_votes[1] = color_votes[2] = color_votes[3] = 0;
                start_time = ros::Time::now();
            } else if (arrived_point == b_point_index) {
                state = State::TO_COLOR_ENDPOINT;
                pub_flag = 1;
                start_time = ros::Time::now();
            } else {
                current_point_idx++;
                pub_flag = 1;
                start_time = ros::Time::now();
            }
        }
    }

    /**
     * 在 A 点：悬停等待，多次采样取众数确定颜色
     */
    void identifyColor() {
        // 悬停采样 2 秒，每帧投票
        if (ros::Time::now() - start_time < ros::Duration(2.0)) {
            if (latest_color >= 1 && latest_color <= 3) {
                color_votes[latest_color]++;
            }
            return; // 等待采样
        }

        // 统计投票结果
        int best = 1, max_votes = color_votes[1];
        for (int i = 2; i <= 3; i++) {
            if (color_votes[i] > max_votes) {
                max_votes = color_votes[i];
                best = i;
            }
        }
        color_store = best;

        ROS_INFO("======================================================");
        ROS_INFO("  [TASK 7] A区识别颜色: >>> %s <<< (votes: R=%d G=%d B=%d)",
                 color_names[color_store].c_str(),
                 color_votes[1], color_votes[2], color_votes[3]);
        ROS_INFO("======================================================");

        current_point_idx++;
        pub_flag = 1;
        start_time = ros::Time::now();
        state = State::NAVIGATING;
    }

    /**
     * 飞向颜色对应的终点站（加超时保护：15秒）
     */
    void toColorEndpoint() {
        if (pub_flag) {
            ROS_INFO("[TO_COLOR_ENDPOINT] Flying to %s endpoint (%.2f, %.2f)",
                     color_names[color_store].c_str(),
                     color_endpoints[color_store].x,
                     color_endpoints[color_store].y);
            goal_with_id.drone_id = 0;
            goal_with_id.goal[0] = color_endpoints[color_store].x;
            goal_with_id.goal[1] = color_endpoints[color_store].y;
            goal_with_id.goal[2] = fly_height;
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }

        bool arrived = (distance() < tolerance);
        bool timeout = (ros::Time::now() - start_time > ros::Duration(15.0));

        if (arrived || timeout) {
            if (timeout && !arrived) {
                ROS_WARN("[TO_COLOR_ENDPOINT] Timeout, forcing advance");
            } else {
                ROS_INFO("[TO_COLOR_ENDPOINT] Arrived at %s endpoint", color_names[color_store].c_str());
            }
            state = State::VERIFY_COLOR;
            start_time = ros::Time::now();
        }
    }

    /**
     * 在终点验证颜色：悬停1秒后采样验证
     */
    void verifyColor() {
        // 悬停等待 1 秒让相机稳定
        if (ros::Time::now() - start_time < ros::Duration(1.0)) {
            return;
        }

        int current = latest_color;
        if (current == color_store) {
            ROS_INFO("======================================================");
            ROS_INFO("  [TASK 8] 颜色验证通过! 终点颜色: %s = A区颜色: %s",
                     color_names[current].c_str(), color_names[color_store].c_str());
            ROS_INFO("======================================================");
        } else {
            ROS_WARN("======================================================");
            ROS_WARN("  [TASK 8] 颜色不匹配! 终点: %s, A区记忆: %s",
                     color_names[current].c_str(), color_names[color_store].c_str());
            ROS_WARN("======================================================");
        }

        pub_flag = 1;
        start_time = ros::Time::now();
        state = State::PRELAND;
    }

    void preland() {
        if (pub_flag) {
            std_msgs::Bool stop_msg;
            stop_msg.data = true;
            stop_cmd_pub.publish(stop_msg);
            pub_flag = 0;
            ROS_INFO("[State] PRELAND");
        }
        if (ros::Time::now() - start_time > ros::Duration(4.0)) {
            state = State::LANDING;
        }
    }

    void land() {
        ROS_INFO("[State] LANDING");
        quadrotor_msgs::TakeoffLand msg;
        msg.takeoff_land_cmd = 2;
        takeoff_land_pub.publish(msg);
    }

    double distance() {
        double dx = goal_with_id.goal[0] - position_3d.pose.position.x;
        double dy = goal_with_id.goal[1] - position_3d.pose.position.y;
        return sqrt(dx * dx + dy * dy);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_control_node");
    MissionControl mc;
    ros::Rate rate(5);
    while (ros::ok()) {
        mc.run();
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
