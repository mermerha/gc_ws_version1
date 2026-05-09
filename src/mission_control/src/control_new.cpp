#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/GoalSet.h>
#include <std_srvs/Trigger.h> // [新] 包含 Trigger 消息

using namespace std;

enum class State {
    TAKING_OFF,
    INITIALIZING,
    NAVIGATING,          // 按顺序飞航点
    IDENTIFY_COLOR,      // 在 A 点读取颜色
    GRAB_BALL,           // [新] 抓球
    TO_DROP_POINT,       // [新] 飞向对应颜色投球点
    DESCEND_TO_DROP,     // [新] 下降到投球高度
    DROP_BALL,           // [新] 松爪投球
    TO_COLOR_ENDPOINT,   // 飞向颜色对应的降落终点
    VERIFY_COLOR,        // 在终点验证颜色 (原有功能保留，可选项)
    PRELAND,
    LANDING
};

class MissionControl {
public:
    MissionControl() : state(State::TAKING_OFF), color_store(0),
                       latest_color(0), current_point_idx(0), pub_flag(1), grab_retries(0) {
        odometry_sub = nh.subscribe("/ekf/ekf_odom", 10, &MissionControl::odom_callback, this);
        color_sub = nh.subscribe("/detected_color", 1, &MissionControl::color_callback, this);
        goal_pub = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
        takeoff_land_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
        stop_cmd_pub = nh.advertise<std_msgs::Bool>("/stop_cmd", 10);
        
        // [新] 机械爪控制 Service Clients
        claw_grab_client = nh.serviceClient<std_srvs::Trigger>("/claw/grab");
        claw_release_client = nh.serviceClient<std_srvs::Trigger>("/claw/release");

        // 通用参数
        nh.param<float>("/mission_control/tolerance", tolerance, 0.3);
        // fly_height 被每个航点的独立高度取代了，这里作为 fallback
        float fallback_height;
        nh.param<float>("/mission_control/fly_height", fallback_height, 1.2);
        nh.param<int>("/mission_control/num_points", num_points, 5);

        // A 点和 B 点的索引 (1-indexed，和 yaml 里 point1, point2... 对应)
        nh.param<int>("/mission_control/a_point_index", a_point_index, 2);
        nh.param<int>("/mission_control/b_point_index", b_point_index, 5);
        nh.param<int>("/mission_control/default_color", default_color, 1);

        // 读取所有航点及独立高度
        for (int i = 0; i < num_points && i < 15; ++i) {
            string name = "point" + to_string(i + 1);
            nh.param<float>("/mission_control/" + name + "_x", waypoints[i].x, 0.0);
            nh.param<float>("/mission_control/" + name + "_y", waypoints[i].y, 0.0);
            nh.param<float>("/mission_control/" + name + "_z", waypoint_heights[i], fallback_height);
        }

        // [新] 三种颜色的投球点坐标 (索引: 1=Red, 2=Green, 3=Blue)
        nh.param<float>("/mission_control/drop_r_x", drop_points[1].x, 0.0);
        nh.param<float>("/mission_control/drop_r_y", drop_points[1].y, 0.0);
        nh.param<float>("/mission_control/drop_r_z", drop_point_heights[1], fallback_height);
        
        nh.param<float>("/mission_control/drop_g_x", drop_points[2].x, 0.0);
        nh.param<float>("/mission_control/drop_g_y", drop_points[2].y, 0.0);
        nh.param<float>("/mission_control/drop_g_z", drop_point_heights[2], fallback_height);
        
        nh.param<float>("/mission_control/drop_b_x", drop_points[3].x, 0.0);
        nh.param<float>("/mission_control/drop_b_y", drop_points[3].y, 0.0);
        nh.param<float>("/mission_control/drop_b_z", drop_point_heights[3], fallback_height);
        
        // 投球时下降高度
        nh.param<float>("/mission_control/drop_height", drop_height, 0.6);

        // [修改] 三种颜色的降落终点坐标
        nh.param<float>("/mission_control/land_r_x", color_endpoints[1].x, 0.0);
        nh.param<float>("/mission_control/land_r_y", color_endpoints[1].y, 0.0);
        nh.param<float>("/mission_control/land_r_z", land_point_heights[1], fallback_height);
        nh.param<float>("/mission_control/land_g_x", color_endpoints[2].x, 0.0);
        nh.param<float>("/mission_control/land_g_y", color_endpoints[2].y, 0.0);
        nh.param<float>("/mission_control/land_g_z", land_point_heights[2], fallback_height);
        nh.param<float>("/mission_control/land_b_x", color_endpoints[3].x, 0.0);
        nh.param<float>("/mission_control/land_b_y", color_endpoints[3].y, 0.0);
        nh.param<float>("/mission_control/land_b_z", land_point_heights[3], fallback_height);

        ROS_INFO("[MissionControl] num_points=%d, A=point%d, B=point%d, default_color=%d", 
                 num_points, a_point_index, b_point_index, default_color);
    }

    void run() {
        switch (state) {
            case State::TAKING_OFF:       takeOff(); break;
            case State::INITIALIZING:     initialize(); break;
            case State::NAVIGATING:       navigate(); break;
            case State::IDENTIFY_COLOR:   identifyColor(); break;
            case State::GRAB_BALL:        grabBall(); break;
            case State::TO_DROP_POINT:    toDropPoint(); break;
            case State::DESCEND_TO_DROP:  descendToDrop(); break;
            case State::DROP_BALL:        dropBall(); break;
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
    ros::ServiceClient claw_grab_client, claw_release_client;

    geometry_msgs::PoseStamped position_3d;
    quadrotor_msgs::GoalSet goal_with_id;
    ros::Time start_time;
    int pub_flag;
    int grab_retries;

    // 航点
    Point waypoints[15];
    float waypoint_heights[15];
    int num_points;
    int current_point_idx;   // 0-indexed

    // A/B 索引 (1-indexed)
    int a_point_index;
    int b_point_index;
    int default_color;

    // 颜色
    int latest_color;        // 来自话题的实时颜色
    int color_store;         // 在 A 点记住的颜色
    int color_votes[4];      // 颜色投票计数 [0~3]
    
    // [新] 投球与降落坐标
    Point drop_points[4];
    float drop_point_heights[4];
    float drop_height;
    Point color_endpoints[4]; 
    float land_point_heights[4]; // [新] 降落点独立高度
    string color_names[4] = {"UNKNOWN", "RED", "GREEN", "BLUE"};

    float tolerance;

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
     */
    void navigate() {
        if (current_point_idx >= num_points) {
            state = State::PRELAND;
            start_time = ros::Time::now();
            return;
        }

        // 发布当前目标航点
        if (pub_flag) {
            ROS_INFO("[NAVIGATING] Flying to point %d (%.2f, %.2f, %.2f)",
                     current_point_idx + 1,
                     waypoints[current_point_idx].x,
                     waypoints[current_point_idx].y,
                     waypoint_heights[current_point_idx]);
            goal_with_id.drone_id = 0;
            goal_with_id.goal[0] = waypoints[current_point_idx].x;
            goal_with_id.goal[1] = waypoints[current_point_idx].y;
            goal_with_id.goal[2] = waypoint_heights[current_point_idx]; // [修改] 独立高度
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
                state = State::TO_DROP_POINT; // [修改] 到达B点后，下一步是飞去投球点
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
        // [优化] 延长采样时间至 4 秒
        if (ros::Time::now() - start_time < ros::Duration(4.0)) {
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
        
        // [新] 如果没有有效投票，使用默认颜色
        int total = color_votes[1] + color_votes[2] + color_votes[3];
        if (total == 0) {
            color_store = default_color;
            ROS_WARN("[IDENTIFY] No valid votes! Using default fallback: %s", color_names[default_color].c_str());
        } else {
            color_store = best;
            ROS_INFO("======================================================");
            ROS_INFO("  [TASK 7] A区识别颜色: >>> %s <<< (votes: R=%d G=%d B=%d)",
                     color_names[color_store].c_str(),
                     color_votes[1], color_votes[2], color_votes[3]);
            ROS_INFO("======================================================");
        }

        // 满足规则：A区停留满5秒
        if (ros::Time::now() - start_time < ros::Duration(5.0)) return;

        state = State::GRAB_BALL;
        pub_flag = 1;
        grab_retries = 0; // 重置重试次数
        start_time = ros::Time::now();
    }

    /**
     * [新] 抓球逻辑
     */
    void grabBall() {
        if (pub_flag) {
            ROS_INFO("[GRAB_BALL] Calling /claw/grab to grab the ball...");
            std_srvs::Trigger srv;
            if (claw_grab_client.call(srv) && srv.response.success) {
                ROS_INFO("[GRAB_BALL] Grab success!");
                pub_flag = 0;
                start_time = ros::Time::now(); // 重置时间以等待动作完成
            } else {
                ROS_WARN("[GRAB_BALL] Grab failed or service unavailable: %s", srv.response.message.c_str());
                grab_retries++;
                if (grab_retries >= 3) {
                    ROS_ERROR("[GRAB_BALL] Failed 3 times, skipping grab and continuing mission.");
                    pub_flag = 0;
                    start_time = ros::Time::now();
                } else {
                    ROS_WARN("[GRAB_BALL] Retrying... (%d/3)", grab_retries);
                    ros::Duration(0.5).sleep(); // 短暂延时后重试
                    return; // 下一帧继续重试
                }
            }
        }

        // 等待机械爪动作完成 (3秒)
        if (ros::Time::now() - start_time > ros::Duration(3.0)) {
            current_point_idx++;  // 继续下一个航点
            pub_flag = 1;
            start_time = ros::Time::now();
            state = State::NAVIGATING;
        }
    }

    /**
     * [新] 飞向投球点
     */
    void toDropPoint() {
        if (pub_flag) {
            ROS_INFO("[TO_DROP_POINT] Flying to %s drop point (%.2f, %.2f, %.2f)",
                     color_names[color_store].c_str(),
                     drop_points[color_store].x,
                     drop_points[color_store].y,
                     drop_point_heights[color_store]);
            goal_with_id.drone_id = 0;
            goal_with_id.goal[0] = drop_points[color_store].x;
            goal_with_id.goal[1] = drop_points[color_store].y;
            goal_with_id.goal[2] = drop_point_heights[color_store];
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }

        bool arrived = (distance() < tolerance);
        bool timeout = (ros::Time::now() - start_time > ros::Duration(15.0));

        if (arrived || timeout) {
            if (timeout && !arrived) {
                ROS_WARN("[TO_DROP_POINT] Timeout, forcing advance");
            } else {
                ROS_INFO("[TO_DROP_POINT] Arrived at %s drop point", color_names[color_store].c_str());
            }
            state = State::DESCEND_TO_DROP;
            pub_flag = 1;
            start_time = ros::Time::now();
        }
    }

    /**
     * [新] 下降到投球高度
     */
    void descendToDrop() {
        if (pub_flag) {
            ROS_INFO("[DESCEND] Lowering to drop height %.2f", drop_height);
            goal_with_id.goal[2] = drop_height;  // 修改高度，X和Y保持不变
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
            start_time = ros::Time::now();
        }

        // 判断高度是否到位 (容差 0.10m)，或超时保护
        double dz = fabs(position_3d.pose.position.z - drop_height);
        bool at_height = (dz < 0.10);
        bool timeout = (ros::Time::now() - start_time > ros::Duration(8.0));

        if (at_height || timeout) {
            state = State::DROP_BALL;
            pub_flag = 1;
            start_time = ros::Time::now();
        }
    }

    /**
     * [新] 松爪投球
     */
    void dropBall() {
        if (pub_flag) {
            ROS_INFO("[DROP_BALL] Calling /claw/release to drop ball...");
            std_srvs::Trigger srv;
            if (claw_release_client.call(srv) && srv.response.success) {
                ROS_INFO("[DROP_BALL] Release success!");
            } else {
                ROS_WARN("[DROP_BALL] Release failed: %s", srv.response.message.c_str());
            }
            pub_flag = 0;
            start_time = ros::Time::now();
        }

        // 等待 2 秒确保球脱落
        if (ros::Time::now() - start_time > ros::Duration(2.0)) {
            state = State::TO_COLOR_ENDPOINT;
            pub_flag = 1;
            start_time = ros::Time::now();
        }
    }

    /**
     * 飞向颜色对应的降落终点站
     */
    void toColorEndpoint() {
        if (pub_flag) {
            ROS_INFO("[TO_COLOR_ENDPOINT] Flying to %s land point (%.2f, %.2f)",
                     color_names[color_store].c_str(),
                     color_endpoints[color_store].x,
                     color_endpoints[color_store].y);
            goal_with_id.drone_id = 0;
            goal_with_id.goal[0] = color_endpoints[color_store].x;
            goal_with_id.goal[1] = color_endpoints[color_store].y;
            // [修改] 使用专属的降落点高度
            goal_with_id.goal[2] = land_point_heights[color_store]; 
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }

        bool arrived = (distance() < tolerance);
        bool timeout = (ros::Time::now() - start_time > ros::Duration(15.0));

        if (arrived || timeout) {
            if (timeout && !arrived) {
                ROS_WARN("[TO_COLOR_ENDPOINT] Timeout, forcing advance");
            } else {
                ROS_INFO("[TO_COLOR_ENDPOINT] Arrived at %s land point", color_names[color_store].c_str());
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
            ROS_INFO("  [TASK 8] 颜色验证通过! 终点颜色: %s = A区记忆: %s",
                     color_names[current].c_str(), color_names[color_store].c_str());
            ROS_INFO("======================================================");
        } else {
            ROS_WARN("======================================================");
            ROS_WARN("  [TASK 8] 颜色不匹配或未识别! 终点: %s, A区记忆: %s",
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
