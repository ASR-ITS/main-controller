#include <ros/ros.h>
#include "control_layout.h"

Robot::Robot(): RosRate(100)
{
    // Initialize
    ROS_INFO("Robot Main Controller");
    
    // Subscriber & Publisher
    SubJoy          = Nh.subscribe("/joy", 18, &Robot::JoyCallback, this);
    SubJoyBattery   = Nh.subscribe("/battery", 10, &Robot::JoyBatteryCallback, this);
    PubSpeed        = Nh.advertise<main_controller::ControllerData>("robot/cmd_vel", 10);
    PubJoyFeedback  = Nh.advertise<sensor_msgs::JoyFeedbackArray>("/set_feedback", 10);

    // Initialize Speed Variable
    for (int i = 0; i <=2 ; i++){
        MsgSpeed.data.push_back(0);
    }

    // Initialize Joystick LED Feedback
    MsgJoyLED_R.type    = 0;
    MsgJoyLED_R.id      = 0;
    MsgJoyLED_R.intensity = 0.0;

    MsgJoyLED_G.type    = 0;
    MsgJoyLED_G.id      = 1;
    MsgJoyLED_R.intensity = 0.0;

    MsgJoyLED_B.type    = 0;
    MsgJoyLED_B.id      = 2;
    MsgJoyLED_B.intensity = 0.0;

    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_R);
    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_G);
    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_B);

    while(ros::ok()){
        // Set Status Control using Triangle Button
        if (Controller.Buttons[triangle] == 0 && Controller.prev_button[triangle] == 1)
        {
            StatusControl ^= 1;
            MsgSpeed.StatusControl = StatusControl;
        }
        Controller.prev_button[triangle] = Controller.Buttons[triangle];

        // Print Robot Speed to Screen
        std::cout << "Speed[0] : " << RobotSpeed[0] << " Speed[1] : " << RobotSpeed[1] << " Speed[2] : " << RobotSpeed[2] << " Status : " << MsgSpeed.StatusControl << " Joystick Battery : " << JoyBatt*100 << "%" << std::endl;

        if (Controller.Buttons[OPTIONS] == 0 && Controller.prev_button[OPTIONS] == 1)
        {
            GuidedMode ^= 1;
        }
        Controller.prev_button[OPTIONS] = Controller.Buttons[OPTIONS];

        if (GuidedMode)
        {
            // Set LED Feedback PURPLE for Guided Mode
            MsgJoyLED_R.intensity = 0.13;
            MsgJoyLED_G.intensity = 0.1;
            MsgJoyLED_B.intensity = 0.53;

            // ***TO-DO : ADD PURE PURSUIT MODE*** //
            // Set Robot Speed to Zero (CHECK AGAIN FOR PURE PURSUIT IMPLEMENTATION)
            for(int i = 0; i<3; i++)
            {
                RobotSpeed[i] = 0;
            }
        }
        else
        {
            if (MsgSpeed.StatusControl)
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
                for(int i = 0; i<3; i++)
                {
                    RobotSpeed[i] = 0;
                }     
            }
        }

        // Push Robot Variable to Publisher Var
        for(int i = 0 ; i<=2 ; i++)
        {
            MsgSpeed.data.at(i) = RobotSpeed[i];
        }

        MsgJoyFeedbackArray.array.at(0) = MsgJoyLED_R;
        MsgJoyFeedbackArray.array.at(1) = MsgJoyLED_G;
        MsgJoyFeedbackArray.array.at(2) = MsgJoyLED_B;

        // Publish Topics
        PubJoyFeedback.publish(MsgJoyFeedbackArray);
        PubSpeed.publish(MsgSpeed);

        ros::spinOnce(); 
        RosRate.sleep();
    }
};

Robot::~Robot(){}

void Robot::JoyCallback(const sensor_msgs::Joy::ConstPtr &MsgJoy)
{
    for(int i = 0; i<4; i++)
    {
        Controller.Axis[i] = MsgJoy->axes[i];
    }

    for(int i = 0; i<18; i++)
    {
        Controller.Buttons[i] = MsgJoy->buttons[i];
    }
}

void Robot::JoyBatteryCallback(const sensor_msgs::BatteryState::ConstPtr &MsgJoyBatt)
{
    JoyBatt = MsgJoyBatt->percentage;
}

void Robot::OdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
}