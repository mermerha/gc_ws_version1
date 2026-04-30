#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <iostream>
#include <string>
#include <std_msgs/Int32.h>
#include "mavros_msgs/RCIn.h"
// #include <mission_control/setpoint.h>
// #include <mission_control/takeland.h>
#include <opencv_detect/Color_detection.h>
// #include <mission_control/catchdropball.h>

#include <mavros_msgs/CommandBool.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/GoalSet.h>
using namespace std;

#define U1 320
#define V1 240

#define THRESHOLD_U 2
#define THRESHOLD_V 2

#define KP_X 0.05
#define KP_Y 0.05

enum class State
{

    TAKING_OFF,
    INITIALIZING,
    TO_POINT_1,
    TO_POINT_2,
    TO_POINT_3,
    TO_POINT_4,
    CATCH_BALL,
    UP,
    IDENTIFY_COLOR,
    TO_POINT_5,
    TO_POINT_6,
    TO_POINT_7,
    TO_POINT_8,
    DETECT_COLOR1,
    DETECT_COLOR2,
    TO_POINT_10,
    DROP_BALL,
    PRELAND,
    LANDING
};

class MissionControl
{
public:
    MissionControl() : state(State::TAKING_OFF)
    {

        odometry_sub = nh.subscribe("/ekf/ekf_odom", 10, &MissionControl::odemetry_callback, this);
        // pos_sub = nh.subscribe("/temp/pixel", 10, &MissionControl::pixel_callback, this);
        goal_pub = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
        takeoff_land_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
        // motor_pub = nh.advertise<mission_control::motor>("/motor_msgs", 10);
        catch_pub = nh.advertise<std_msgs::Int32>("/catch", 10);
        drop_pub = nh.advertise<std_msgs::Int32>("/release", 10);
        color_client = nh.serviceClient<opencv_detect::Color_detection>("/detect_color",10);

        nh.param<float>("/mission_control/tolerance", tolerance, 0.1);

        nh.param<float>("/mission_control/fly_height", fly_height, 0.0);
        nh.param<float>("/mission_control/fly_height_color", fly_height_color, 0.0);
        nh.param<float>("/mission_control/fly_height_ball", fly_height_ball, 0.0);

        nh.param<float>("/mission_control/point1_x", point1_x, 0.0);
        nh.param<float>("/mission_control/point1_y", point1_y, 0.0);

        nh.param<float>("/mission_control/point2_x", point2_x, 0.0);
        nh.param<float>("/mission_control/point2_y", point2_y, 0.0);

        nh.param<float>("/mission_control/point3_x", point3_x, 0.0);
        nh.param<float>("/mission_control/point3_y", point3_y, 0.0);

        nh.param<float>("/mission_control/point4_x", point4_x, 0.0);
        nh.param<float>("/mission_control/point4_y", point4_y, 0.0);

        nh.param<float>("/mission_control/point5_x", point5_x, 0.0);
        nh.param<float>("/mission_control/point5_y", point5_y, 0.0);

        nh.param<float>("/mission_control/point6_x", point6_x, 0.0);
        nh.param<float>("/mission_control/point6_y", point6_y, 0.0);

        nh.param<float>("/mission_control/point7_x", point7_x, 0.0);
        nh.param<float>("/mission_control/point7_y", point7_y, 0.0);

        nh.param<float>("/mission_control/land_point_r_x", point8_x, 0.0);
        nh.param<float>("/mission_control/land_point_r_y", point8_y, 0.0);

        nh.param<float>("/mission_control/land_point_g_x", point9_x, 0.0);
        nh.param<float>("/mission_control/land_point_g_y", point9_y, 0.0);

        nh.param<float>("/mission_control/land_point_b_x", point10_x, 0.0);
        nh.param<float>("/mission_control/land_point_b_y", point10_y, 0.0);

        nh.param<float>("/mission_control/drop_point_r_x", droppoint8_x, 0.0);
        nh.param<float>("/mission_control/drop_point_r_y", droppoint8_y, 0.0);

        nh.param<float>("/mission_control/drop_point_g_x", droppoint9_x, 0.0);
        nh.param<float>("/mission_control/drop_point_g_y", droppoint9_y, 0.0);

        nh.param<float>("/mission_control/drop_point_b_x", droppoint10_x, 0.0);
        nh.param<float>("/mission_control/drop_point_b_y", droppoint10_y, 0.0);

    }

    void run()
    {
        switch (state)
        {
        case State::TAKING_OFF:
            takeOff();
            break;
        case State::INITIALIZING:
            initialize();
            break;
        case State::TO_POINT_1:
            topoint1();
            break;

        case State::TO_POINT_2:
            topoint2();
            break;

        case State::TO_POINT_3:
            topoint3();
            break;

        case State::TO_POINT_4:
            topoint4();
            break;

        case State::CATCH_BALL:
            catchball();
            break;

        case State::UP:
            up();
            break;

        case State::IDENTIFY_COLOR:
            identifycolor();
            break;

        case State::TO_POINT_5:
            topoint5();
            break;

        // case State::DROPPING:
        //     drop();
        //     break;

        case State::TO_POINT_6:
            topoint6();
            break;

        case State::TO_POINT_7:
            topoint7();
            break;

        case State::TO_POINT_8:
            topoint8();
            break;

        // case State::DETECT_COLOR1:
        //     detectcolor1();
        //     break;

        // case State::DETECT_COLOR2:
        //     detectcolor2();
        //     break;

        case State::TO_POINT_10:
            topoint10();
            break;

        case State::DROP_BALL:
            dropball();
            break;

        case State::PRELAND:
            preland();
            break;

        case State::LANDING:
            land();
            break;

        default:
            break;
        }
    }

    void takeOff()
    {
        ROS_INFO("Taking off...");
        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 1; // 设置起飞命令
        if (takeoff_land_pub.getNumSubscribers() > 0)
        {
            takeoff_land_pub.publish(takeoff_msg);
            start_time = ros::Time::now();
            state = State::INITIALIZING; // 假设起飞后直接进入导航状态
        }
    }

    void initialize()
    {
        ROS_INFO("Initialing...");
        // rc_in_data_sub = nh.subscribe("/mavros/rc/in", 10, rc_in_data_callback);
        // spinOnce();
        // if (rc_in_data)
        //     ROS_INFO("Initializing...");
        // int ready=0;
        // ros::param::get("/ready",ready);
        // if(ready)
        if (ros::Time::now() - start_time < ros::Duration(3.0))
        {
            return;
        }
        drop.data = 1;
        drop_pub.publish(drop);
        pub_flag = 1;
        state = State::TO_POINT_1; // 假设初始化后直接进入起飞状态
    }

    void topoint1()
    {
        ROS_INFO("Flying to point1...");
        if (goal_pub.getNumSubscribers() > 0)
        {
            if(pub_flag){
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = point1_x;
                goal_with_id.goal[1] = point1_y;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::TO_POINT_2;
            }
        }
    }

    void topoint2()
    {
        ROS_INFO("Flying to point2...");
        if (goal_pub.getNumSubscribers() > 0)
        {
            if(pub_flag){
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = point2_x;
                goal_with_id.goal[1] = point2_y;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::TO_POINT_3;
            }
        }
    }

    void topoint3()
    {
        ROS_INFO("Flying to point3...");

        if (goal_pub.getNumSubscribers() > 0)
        {
            if(pub_flag){
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = point3_x;
                goal_with_id.goal[1] = point3_y;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::TO_POINT_4;
            }
        }
    }
    
    void topoint4()
    {
        ROS_INFO("Flying to point4...");

        if (goal_pub.getNumSubscribers() > 0)
        {
            if(pub_flag){
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = point4_x;
                goal_with_id.goal[1] = point4_y;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::CATCH_BALL;
            }
        }
        
    }

    //要加高度调整的状态 down()

    void catchball()
    {
        ROS_INFO("Catching ball...");
        if(pub_flag){
            goal_with_id.goal[2] = fly_height_ball;
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }
        if ((ros::Time::now() - start_time > ros::Duration(5.0))||distance() < tolerance){
            cat.data = 1;
            catch_pub.publish(cat); //需要拖一点时间?
            
            if(ros::Time::now()-start_time > ros::Duration(8.0)){
                pub_flag = 1;
                start_time = ros::Time::now();
                state = State::UP;
            }
        }
    }

    void up(){
        ROS_INFO("Going up and recognizing color...");
        if(pub_flag){
            goal_with_id.goal[2] = fly_height;
            goal_pub.publish(goal_with_id);
            pub_flag = 0;
        }
        if ((ros::Time::now() - start_time > ros::Duration(5.0))||distance() < tolerance){
            opencv_detect::Color_detection srv;
            srv.request.detection_request = 1;
            if (color_client.call(srv))
            {
                color_store = srv.response.color;
                ROS_INFO_STREAM("colorstore=" << col[color_store]);  //需要延时循环吗?
                start_time = ros::Time::now();
                pub_flag=1;
                state = State::TO_POINT_5;
            }
        }
    }

    void identifycolor()
    {
        ROS_INFO("identifying color...");

        if (distance() < tolerance)
        {
            // nh.setParam("/detect", 1);
            opencv_detect::Color_detection srv;
            srv.request.detection_request = 1;
            if (color_client.call(srv))
            {
                color_store = srv.response.color;
                ROS_INFO_STREAM("colorstore=" << color_store);  //需要延时循环吗?
            }
            if(ros::Time::now()-start_time > ros::Duration(5.0)){
                start_time = ros::Time::now();
                state = State::IDENTIFY_COLOR;
            }
            state = State::TO_POINT_5;
            
        }
    }

    void topoint5()
    {
        ROS_INFO("Flying to point5...");
        
        if (goal_pub.getNumSubscribers() > 0)
        {
            if(pub_flag){
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = point5_x;
                goal_with_id.goal[1] = point5_y;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::TO_POINT_6;
            }
        }
            
    }

    void topoint6()
    {
        ROS_INFO("Flying to point6...");

        if (goal_pub.getNumSubscribers() > 0)
        {
            if(pub_flag){
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = point6_x;
                goal_with_id.goal[1] = point6_y;
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::TO_POINT_7;
            }
        }
    }

    void topoint7()
    {
        ROS_INFO("Flying to point7...");

        if (distance() < tolerance)
        {
            if (goal_pub.getNumSubscribers() > 0)
            {
                if(pub_flag){
                    goal_with_id.drone_id = 0;
                    goal_with_id.goal[0] = point7_x;
                    goal_with_id.goal[1] = point7_y;
                    goal_with_id.goal[2] = fly_height;
                    goal_pub.publish(goal_with_id);
                    pub_flag = 0;
                }
                if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
                    start_time = ros::Time::now();
                    pub_flag = 1;
                    state = State::TO_POINT_10;
                }
            }
        }
    }

    void topoint8()
    {
        ROS_INFO("Flying to point8...");

        if (goal_pub.getNumSubscribers() > 0)
        {
            if(pub_flag){
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = pointcolor_x[color_store];
                goal_with_id.goal[1] = pointcolor_y[color_store];
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::PRELAND;
            }
        }
    }

    // void detectcolor1()
    // {
    //     ROS_INFO("Detecting color 1...");
    //     if (ros::Time::now() - start_time < ros::Duration(3.0))
    //         return;
    //     if (ros::Time::now() - start_time < ros::Duration(4.0))
    //         return;
    //     opencv_detect::Color_detection srv;
    //     srv.request.detection_request = 1;
    //     if (color_client.call(srv))
    //     {
    //         color_detect = srv.response.color;
    //     }
    //     if (color_detect == color_store)
    //     {
    //         state = State::DROP_BALL;
    //     }
    //     else
    //     {
    //         goal_with_id.drone_id = 0;
    //         goal_with_id.goal[0] = point9_x;
    //         goal_with_id.goal[1] = point9_y;
    //         goal_with_id.goal[2] = fly_height;
    //         goal_pub.publish(goal_with_id);
    //         if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
    //             state = State::DETECT_COLOR2;
    //         }
    //     }
    // }

    // void detectcolor2()
    // {
    //     ROS_INFO("Detecting color 2...");
    //     if (ros::Time::now() - start_time < ros::Duration(3.0))
    //         return;
    //     if (ros::Time::now() - start_time < ros::Duration(4.0))
    //         return;
    //     opencv_detect::Color_detection srv;
    //     srv.request.detection_request = 1;
    //     if (color_client.call(srv))
    //     {
    //         color_detect = srv.response.color;
    //     }
    //     if (color_detect == color_store)
    //     {
    //         state = State::DROP_BALL;
    //     }
    //     else
    //     {
    //         goal_with_id.drone_id = 0;
    //         goal_with_id.goal[0] = point10_x;
    //         goal_with_id.goal[1] = point10_y;
    //         goal_with_id.goal[2] = fly_height_color;
    //         goal_pub.publish(goal_with_id);

    //         state = State::TO_POINT_10;
    //     }
    // }

    void topoint10()
    {
        ROS_INFO("Flying to point8...");

        if (goal_pub.getNumSubscribers() > 0)
        {
            if(pub_flag){
                goal_with_id.drone_id = 0;
                goal_with_id.goal[0] = pointdrop_x[color_store];
                goal_with_id.goal[1] = pointdrop_y[color_store];
                goal_with_id.goal[2] = fly_height;
                goal_pub.publish(goal_with_id);
                pub_flag = 0;
            }
            if ((ros::Time::now() - start_time > ros::Duration(15.0))||distance() < tolerance){
                start_time = ros::Time::now();
                pub_flag = 1;
                state = State::DROP_BALL;
            }
        }
    }

    void dropball()
    {
        ROS_INFO("dropping...");
        drop.data = 1;
        drop_pub.publish(drop);
        if(ros::Time::now()-start_time > ros::Duration(3.0)){
            start_time = ros::Time::now();
            state = State::TO_POINT_8;
        }
    }

    void preland()
    {
        ROS_INFO("Prelanding...");
        if (distance() > tolerance)
            return;
        state = State::LANDING;
    }

    void land()
    {
        ROS_INFO("Landing...");
        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 2; // 设置起飞命令
        if (takeoff_land_pub.getNumSubscribers() > 0)
        {
            takeoff_land_pub.publish(takeoff_msg);
            pub_flag = 0;
        }
    }

    // void rc_in_data_callback(const mavros_msgs::RCIn::ConstPtr &msg)
    // {
    //     if (!msg->channels.empty())
    //     {
    //         rc_in_data = msg->channels[5];
    //     }
    // }

    double distance()
    {
        double dx = goal_with_id.goal[0] - position_3d.pose.position.x;
        double dy = goal_with_id.goal[1] - position_3d.pose.position.y;
        // double dz = goal.pose.position.z - position_3d.pose.position.z;
        // return sqrt(dx * dx + dy * dy + dz * dz);
        return sqrt(dx * dx + dy * dy);
    }

    // void pixel_callback(const mission_control::pixel::ConstPtr &pMsg)
    // {

    //     U0 = pMsg->x;
    //     V0 = pMsg->y;
    // }
    void odemetry_callback(const nav_msgs::Odometry::ConstPtr &pMsg)
    {
        odem_msg = *pMsg;
        position_3d.pose.position.x = odem_msg.pose.pose.position.x;
        position_3d.pose.position.y = odem_msg.pose.pose.position.y;
        position_3d.pose.position.z = odem_msg.pose.pose.position.z;
        position_3d.pose.orientation.w = 1.0;
    }

private:
    State state;
    int U0, V0;
    ros::NodeHandle nh;

    ros::Publisher takeoff_land_pub;
    ros::Publisher goal_pub;
    ros::Publisher catch_pub;
    ros::Publisher drop_pub;
    // ros::Publisher motor_pub;

    // ros::Subscriber odemetry_sub;
    // ros::Subscriber pos_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber rc_in_data_sub;

    ros::ServiceClient color_client;

    geometry_msgs::PoseStamped position_3d;
    nav_msgs::Odometry odem_msg;

    quadrotor_msgs::GoalSet goal_with_id;
    geometry_msgs::PoseStamped goal_store;
    std_msgs::Int32 cat;          // 0:nomove 1:catch
    std_msgs::Int32 drop;          // 0:nomove 1:drop
    // std_msgs::Int32 detect;         // 0:nomove 1:recognize
    // std_msgs::Int32 detection;      // 0:cannot recognize 1:red 2:green 3:blue

    ros::Time start_time;
    ros::Time drop_time;

    int iswait;
    int isgrab;
    int isdrop;
    int isdetect;
    int color_detect;
    int rc_in_data;
    int color_store;
    int pub_flag;
    string col[4]={"0","red","green","blue"};

    float tolerance;

    float fly_height;
    float fly_height_color;
    float fly_height_ball;

    float point1_x;
    float point1_y;

    float point2_x;
    float point2_y;

    float point3_x;
    float point3_y;

    float point4_x;
    float point4_y;

    float point5_x;
    float point5_y;

    float point6_x;
    float point6_y;

    float point7_x;
    float point7_y;

    float point8_x;
    float point8_y;

    float point9_x;
    float point9_y;

    float point10_x;
    float point10_y;

    float droppoint8_x;
    float droppoint8_y;

    float droppoint9_x;
    float droppoint9_y;

    float droppoint10_x;
    float droppoint10_y;

    float pointcolor_x[4]={0, point8_x, point9_x, point10_x};
    float pointcolor_y[4]={0, point8_y, point9_y, point10_y};

    float pointdrop_x[4]={0, droppoint8_x, droppoint9_x, droppoint10_x};
    float pointdrop_y[4]={0, droppoint8_y, droppoint9_y, droppoint10_y};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control_node");
    MissionControl missionControl;
    ros::Rate rate(1);
    ros::param::set("/ready", 0);
    while (ros::ok())
    {
        missionControl.run();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
