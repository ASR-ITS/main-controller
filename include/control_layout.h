//ROS-Libraries
#include <ros/ros.h>
#include <ros/package.h>

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>

#include "main_controller/ControllerData.h"


//STD-Libraries
#include <iostream>
#include <string>
#include "stdio.h"
#include "stdlib.h"

class Robot
{
public:
    Robot();

    ~Robot();

private:
    int     RobotSpeed[3] = {0, 0, 0};
    uint8_t StatusControl = 0;
    uint8_t GuidedMode    = 0;
    float   JoyBatt;
    float   PosisiOdom[3];
    float   OffsetPos[3];
    
    struct DS4_T 
    {
        uint8_t Buttons[18];
        float   Axis[4] = {0, 0, 0, 0};
        uint8_t prev_button[18];
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

    ros::NodeHandle     Nh;
    ros::Subscriber     SubJoy;
    ros::Subscriber     SubJoyBattery;
    ros::Subscriber     OdomSub;
    ros::Publisher      PubSpeed;
    ros::Publisher      PubJoyFeedback;
    ros::Rate           RosRate;

    sensor_msgs::JoyFeedback        MsgJoyLED_R;
    sensor_msgs::JoyFeedback        MsgJoyLED_G;
    sensor_msgs::JoyFeedback        MsgJoyLED_B;
    sensor_msgs::JoyFeedbackArray   MsgJoyFeedbackArray;
    nav_msgs::Odometry              Odom;
    asr_its::ControllerData         MsgSpeed;

    void JoyCallback            (const sensor_msgs::Joy::ConstPtr &msgJoy);
    void JoyBatteryCallback     (const sensor_msgs::BatteryState::ConstPtr &MsgJoyBatt);
    void OdomCallback           (const nav_msgs::OdometryConstPtr &msg);

};