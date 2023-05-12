#include <ros/ros.h>
#include "control_layout.h"

Robot::Robot(): RosRate(100)
{
    // Initialize
    ROS_INFO("Robot Main Controller");
    
    // Subscriber & Publisher
    Sub_Joy            = Nh.subscribe("/joy", 18, &Robot::Joy_Callback, this);
    Sub_Joy_Battery    = Nh.subscribe("/battery", 10, &Robot::Joy_Battery_Callback, this);

    Sub_Path           = Nh.subscribe("/path", 10, &Robot::Path_Callback, this);
    Sub_Pose           = Nh.subscribe("/amcl_pose", 10, &Robot::Pose_Callback, this);

    Pub_Vel            = Nh.advertise<main_controller::ControllerData>("robot/cmd_vel", 10);
    Pub_Joy_Feedback   = Nh.advertise<sensor_msgs::JoyFeedbackArray>("/set_feedback", 10);

    // Initialize Speed Variable
    for (int i = 0; i <=2 ; i++){
        vel_msg.data.push_back(0);
    }

    // Initialize Joystick LED Feedback
    MsgJoyLED_R.type      = 0;
    MsgJoyLED_R.id        = 0;
    MsgJoyLED_R.intensity = 0.0;

    MsgJoyLED_G.type      = 0;
    MsgJoyLED_G.id        = 1;
    MsgJoyLED_R.intensity = 0.0;

    MsgJoyLED_B.type      = 0;
    MsgJoyLED_B.id        = 2;
    MsgJoyLED_B.intensity = 0.0;

    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_R);
    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_G);
    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_B);

    while(ros::ok()){
        
        // Set Status Control using TRIANGLE Button
        if (Controller.Buttons[triangle] == 0 && Controller.prev_button[triangle] == 1)
        {
            StatusControl ^= 1;
            vel_msg.StatusControl = StatusControl;
        }
        Controller.prev_button[triangle] = Controller.Buttons[triangle];

        // Print Robot Speed to Screen
        std::cout << "Speed[0] : " << RobotSpeed[0] << " Speed[1] : " << RobotSpeed[1] << " Speed[2] : " << RobotSpeed[2] << " Status : " << vel_msg.StatusControl << " Joystick Battery : " << JoyBatt*100 << "%" << std::endl;

        // Set Mode Using OPTIONS Button
        if (Controller.Buttons[OPTIONS] == 0 && Controller.prev_button[OPTIONS] == 1)
        {
            GuidedMode ^= 1;
        }
        Controller.prev_button[OPTIONS] = Controller.Buttons[OPTIONS];

        // Go to Autonomous Mode
        if (GuidedMode)
        {
            // Set LED Feedback PURPLE for Guided Mode
            MsgJoyLED_R.intensity = 0.13;
            MsgJoyLED_G.intensity = 0.1;
            MsgJoyLED_B.intensity = 0.53;

            // ***TO-DO : ADD PURE PURSUIT MODE*** //
            // Set Robot Speed to Zero (CHECK AGAIN FOR PURE PURSUIT IMPLEMENTATION)
            for(int i = 0; i<=2; i++)
            {
                RobotSpeed[i] = 0;
            }
        }

        // Go to Manual Control Mode
        else
        {
            if (vel_msg.StatusControl)
            {
                // Set LED Feedback RED for Joystick Lock Mode
                MsgJoyLED_R.intensity = 0.12;
                MsgJoyLED_G.intensity = 0.75;
                MsgJoyLED_B.intensity = 0.13;

                // Set Robot Speed from Joy Axis
                RobotSpeed[0] = -1 * Controller.Axis[0] * 20;
                RobotSpeed[1] = Controller.Axis[1] * 20;
                RobotSpeed[2] = Controller.Axis[2] * 15;
            }
            else
            {
                // Set LED Feedback GREEN for Joystick Run Mode
                MsgJoyLED_R.intensity = 1.0;
                MsgJoyLED_G.intensity = 0.0;
                MsgJoyLED_B.intensity = 0.13;     

                // Set Robot Speed to Zero (CHECK AGAIN FOR PURE PURSUIT IMPLEMENTATION)
                for(int i = 0; i<=2; i++)
                {
                    RobotSpeed[i] = 0;
                }     
            }
        }

        // Push Robot Variable to Publisher Var
        for(int i = 0 ; i<=2 ; i++)
        {
            vel_msg.data.at(i) = RobotSpeed[i];
        }

        MsgJoyFeedbackArray.array.at(0) = MsgJoyLED_R;
        MsgJoyFeedbackArray.array.at(1) = MsgJoyLED_G;
        MsgJoyFeedbackArray.array.at(2) = MsgJoyLED_B;

        // Publish Topics
        Pub_Vel.publish(vel_msg);
        Pub_Joy_Feedback.publish(MsgJoyFeedbackArray);

        ros::spinOnce(); 
        RosRate.sleep();
    }
};

Robot::~Robot(){}

void Robot::Joy_Callback (const sensor_msgs::Joy::ConstPtr &joy_msg)
{
    for(int i = 0; i<4; i++)
    {
        Controller.Axis[i] = joy_msg->axes[i];
    }

    for(int i = 0; i<18; i++)
    {
        Controller.Buttons[i] = joy_msg->buttons[i];
    }
}

void Robot::Joy_Battery_Callback (const sensor_msgs::BatteryState::ConstPtr &joy_batt_msg)
{
    JoyBatt = joy_batt_msg->percentage;
}

void Robot::Path_Callback (const nav_msgs::Path::ConstPtr &path_msg)
{
    for(int i = 0; i < path_msg->poses.size(); ++i)
    {
        path.x.push(path_msg->poses[i].pose.position.x);
        path.y.push(path_msg->poses[i].pose.position.y);
        path.z.push(path_msg->poses[i].pose.position.z);       
    }
}
void Robot::Pose_Callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    robot_pose.x = pose_msg->pose.pose.position.x;
    robot_pose.y = pose_msg->pose.pose.position.y;
    robot_pose.z = pose_msg->pose.pose.position.z;    
}