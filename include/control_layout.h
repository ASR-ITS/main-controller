//ROS-Libraries
#include <ros/ros.h>
#include <ros/package.h>

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>

#include <tf/transform_broadcaster.h>
#include "main_controller/ControllerData.h"


//STD-Libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include "stdio.h"
#include "stdlib.h"
#include "queue"

#define MATH_PI 3.1415926535897932384626433832795

class Robot
{
public:
    Robot();

    ~Robot();

private:
    int     RobotSpeed[3] = {0, 0, 0};
    uint8_t StatusControl = 0;
    uint8_t GuidedMode    = 0;
    float   JoyBatt       = 0.0;
    
    struct DS4_T 
    {
        uint8_t Buttons[18];
        float   Axis[4] = {0, 0, 0, 0};
        uint8_t prev_button[18];
    };

    struct Path_t{
        std::queue<float> x;
        std::queue<float> y;
        std::queue<float> z;
    };

    struct Pose_t{
        float x;
        float y;
        float z;
    };

    typedef enum
    {
        square    = 0,
        triangle  = 1,
        circle    = 2,
        cross     = 3,
        L1        = 4,
        L2        = 5,
        R1        = 6,
        R2        = 7,
        SHARE     = 8,
        OPTIONS   = 9,
        DPadLeft  = 14,
        DPadUp    = 15,
        DPadRight = 16,
        DPadDown  = 17
    } DS4_Button;
    
    DS4_T Controller;
    Path_t Robot_Path;
    Pose_t Robot_Pose, Robot_Pose_2;

    ros::NodeHandle     Nh;
    ros::Subscriber     SubJoy;
    ros::Subscriber     SubJoyBattery;
    ros::Subscriber     SubOdom;
    ros::Publisher      PubSpeed;
    ros::Publisher      PubJoyFeedback;
    ros::Subscriber     Sub_Odom;
    ros::Subscriber     Sub_Joy;
    ros::Subscriber     Sub_Joy_Battery;
    ros::Subscriber     Sub_Path;
    ros::Subscriber     Sub_Pose;

    ros::Publisher      Pub_Vel;
    ros::Publisher      Pub_Joy_Feedback;
    ros::Rate           RosRate;

    sensor_msgs::JoyFeedback        MsgJoyLED_R;
    sensor_msgs::JoyFeedback        MsgJoyLED_G;
    sensor_msgs::JoyFeedback        MsgJoyLED_B;
    sensor_msgs::JoyFeedbackArray   MsgJoyFeedbackArray;
    nav_msgs::Odometry              Odom;
    main_controller::ControllerData vel_msg;

    void ReadPath(std::string fileName, Path_t &path);
    void ReadPathRelative(std::string fileName, Path_t &path, Pose_t robotPose);
    void ClearPath(Path_t &path);
    Pose_t PurePursuit(Pose_t robotPose, Path_t &path, float offset);
    Pose_t PointToPointPID(Pose_t robotPose, Pose_t targetPose, float maxSpeed);

    void JoyCallback            (const sensor_msgs::Joy::ConstPtr &msgJoy);
    void JoyBatteryCallback     (const sensor_msgs::BatteryState::ConstPtr &MsgJoyBatt);
    
    void Odom_Callback          (const nav_msgs::OdometryConstPtr &msg);
    void Joy_Callback           (const sensor_msgs::Joy::ConstPtr &joy_msg);
    void Joy_Battery_Callback   (const sensor_msgs::BatteryState::ConstPtr &joy_batt_msg);
    void Path_Callback          (const nav_msgs::Path::ConstPtr &path_msg);
    void Pose_Callback          (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);
};