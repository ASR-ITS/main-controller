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

    Sub_Crashed        = Nh.subscribe("/crashed", 10, &Robot::Crashed_Status_Callback, this);
    Sub_Obstacle       = Nh.subscribe("/obstacle_detected", 10, &Robot::Obstacle_Status_Callback, this);
    Sub_Obs_Vel        = Nh.subscribe("/obs_vel", 10, &Robot::Obstacle_Vel_Callback, this);

    Pub_Vel            = Nh.advertise<main_controller::ControllerData>("robot/cmd_vel", 10);
    Pub_Joy_Feedback   = Nh.advertise<sensor_msgs::JoyFeedbackArray>("/set_feedback", 10);
    Pub_Pure_Pursuit   = Nh.advertise<geometry_msgs::Twist>("/pure_pursuit_vel", 10);

    // Rumble_Timer       = Nh.createTimer(ros::Duration(0.75), &Robot::Rumble_Event, this);

    // Initialize Speed Variable
    for (int i = 0; i <=2 ; i++)
    {
        vel_msg.data.push_back(0);
    }

    // Initialize Joystick LED Feedback
    MsgJoyLED_R.type        = 0;
    MsgJoyLED_R.id          = 0;
    MsgJoyLED_R.intensity   = 0.0;

    MsgJoyLED_G.type        = 0;
    MsgJoyLED_G.id          = 1;
    MsgJoyLED_R.intensity   = 0.0;

    MsgJoyLED_B.type        = 0;
    MsgJoyLED_B.id          = 2;
    MsgJoyLED_B.intensity   = 0.0;

    MsgJoyRumble.type       = 1;
    MsgJoyRumble.id         = 1;
    MsgJoyRumble.intensity  = 0.0;    

    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_R);
    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_G);
    MsgJoyFeedbackArray.array.push_back(MsgJoyLED_B);
    MsgJoyFeedbackArray.array.push_back(MsgJoyRumble);

    // Initialize Pure Purusit
    Pose_t target_pose;
    Pose_t pure_pursuit_vel;
    bool prev_crashed = 0;

    while(ros::ok())
    {   
        // Set Status Control using TRIANGLE Button
        if (Controller.Buttons[TRIANGLE] == 0 && Controller.prev_button[TRIANGLE] == 1)
        {
            StatusControl ^= 1;
            vel_msg.StatusControl = StatusControl;
        }
        Controller.prev_button[TRIANGLE] = Controller.Buttons[TRIANGLE];

        // Clear Path Generated using CIRCLE Button
        if (Controller.Buttons[CIRCLE] == 0 && Controller.prev_button[CIRCLE] == 1)
        {
            ClearPath(path);
        }
        Controller.prev_button[CIRCLE] = Controller.Buttons[CIRCLE];

        // Print Robot Speed to Screen
        // std::cout << "x : " << robot_vel[0] << " y : " << robot_vel[1] << " Theta : " << robot_vel[2] << " Status : " << vel_msg.StatusControl << " Joystick Battery : " << JoyBatt*100 << "%" << std::endl;

        // Clear Path if Robot CRASHED
        if (prev_crashed == 0 && crashed_status == 1)
        {
            ClearPath(path);
            for(int i = 0; i<=2; i++)
            {
                robot_vel[i] = 0;
            }
        }
        prev_crashed = crashed_status;

        // Set Mode Using OPTIONS Button
        if (Controller.Buttons[OPTIONS] == 0 && Controller.prev_button[OPTIONS] == 1)
        {
            GuidedMode ^= 1;
            // Set Rumble Feedback
            rumble_status = 1;
            MsgJoyRumble.intensity  = 0.5;
            prev_time = ros::Time::now();
        }
        Controller.prev_button[OPTIONS] = Controller.Buttons[OPTIONS];

        // Rumble Feedback Event
        if (rumble_status && ros::Time::now() - prev_time >= ros::Duration(0.75))
        {
            MsgJoyRumble.intensity  = 0.0;
            rumble_status = 0;
        }

        // Go to Autonomous Mode
        if (GuidedMode)
        {
            // Go to AUTONOMOUS Mode with YELLOW Indicator
            if (vel_msg.StatusControl)
            {
                // Set LED Feedback
                MsgJoyLED_R.intensity = 0.3;
                MsgJoyLED_G.intensity = 0.3;
                MsgJoyLED_B.intensity = 0.0;

                // Prevent going to origin if there's no path
                if(path.x.size() <= 0)
                {
                    target_pose = robot_pose;
                }
                // Search for Closest Node
                else
                {
                    target_pose = PurePursuit(robot_pose, path, 0.5);
                }

                // Pure Pursuit PID
                pure_pursuit_vel = PointToPointPID(robot_pose, target_pose, 20);

                // Convert to Velocity Command (x-y is switched because odom is switched)
                robot_vel[0] = (int) pure_pursuit_vel.y * -1;
                robot_vel[1] = (int) pure_pursuit_vel.x;
                robot_vel[2] = 0; // Remove Yaw Control for TESTING Purposes;

                pure_pursuit_msg.linear.x  = robot_vel[0];
                pure_pursuit_msg.linear.y  = robot_vel[1];
                pure_pursuit_msg.angular.z = robot_vel[2];

                // Obstacle Avoidance Control with WHITE Indicator
                if(obstacle_status)
                {
                    // Set LED Feedback
                    MsgJoyLED_R.intensity = 1.0;
                    MsgJoyLED_G.intensity = 1.0;
                    MsgJoyLED_B.intensity = 1.0;

                    robot_vel[0] = obstacle_avoider_vel.x;
                    robot_vel[1] = obstacle_avoider_vel.y;
                    robot_vel[2] = obstacle_avoider_vel.theta;
                }
            }

            // Pause AUTONOMOUS Mode with RED Indicator
            else
            {
                // Set LED Feedback
                MsgJoyLED_R.intensity = 1.0;
                MsgJoyLED_G.intensity = 0.0;
                MsgJoyLED_B.intensity = 0.13;  

                // Set Robot Speed to Zero (Safety Issues)
                for(int i = 0; i<=2; i++)
                {
                    robot_vel[i] = 0;
                }    
            }

        }

        // Go to Manual Control Mode
        else
        {
            // Go to Manual Control RUN Mode with GREEN Indicator
            if (vel_msg.StatusControl)
            {
                // Set LED Feedback
                MsgJoyLED_R.intensity = 0.12;
                MsgJoyLED_G.intensity = 0.75;
                MsgJoyLED_B.intensity = 0.13;

                // Set Robot Speed from Joy Axis
                robot_vel[0] = -1 * Controller.Axis[0] * 20;
                robot_vel[1] = Controller.Axis[1] * 20;
                robot_vel[2] = Controller.Axis[2] * 15;
            }

            // Go to Manual Control LOCK Mode with RED Indicator
            else
            {
                // Set LED Feedback
                MsgJoyLED_R.intensity = 1.0;
                MsgJoyLED_G.intensity = 0.0;
                MsgJoyLED_B.intensity = 0.13;     

                // Set Robot Speed to Zero (Safety Issues)
                for(int i = 0; i<=2; i++)
                {
                    robot_vel[i] = 0;
                }     
            }
        }

        // Push Robot Variable to Publisher Var
        for(int i = 0 ; i<=2 ; i++)
        {
            vel_msg.data.at(i) = robot_vel[i];
        }

        MsgJoyFeedbackArray.array.at(0) = MsgJoyLED_R;
        MsgJoyFeedbackArray.array.at(1) = MsgJoyLED_G;
        MsgJoyFeedbackArray.array.at(2) = MsgJoyLED_B;
        MsgJoyFeedbackArray.array.at(3) = MsgJoyRumble;

        // Publish Topics
        Pub_Vel.publish(vel_msg);
        Pub_Pure_Pursuit.publish(pure_pursuit_msg);
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
        path.theta.push(path_msg->poses[i].pose.position.z);       
    }
}

void Robot::Pose_Callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    tf::Quaternion q(
        pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y,
        pose_msg->pose.pose.orientation.z,
        pose_msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    robot_pose.x = pose_msg->pose.pose.position.x;
    robot_pose.y = pose_msg->pose.pose.position.y;   
    robot_pose.theta = yaw;
}

void Robot::Obstacle_Status_Callback (const std_msgs::Bool::ConstPtr &obs_status_msg)
{
    obstacle_status = obs_status_msg->data;
}

void Robot::Crashed_Status_Callback (const std_msgs::Bool::ConstPtr &crashed_msg)
{
    crashed_status = crashed_msg->data;
}

void Robot::Obstacle_Vel_Callback    (const geometry_msgs::Twist::ConstPtr &obs_vel_msg)
{
    obstacle_avoider_vel.x = obs_vel_msg->linear.x;
    obstacle_avoider_vel.y = obs_vel_msg->linear.y;
    obstacle_avoider_vel.theta = obs_vel_msg->angular.z;
}

// void Robot::Rumble_Event (const ros::TimerEvent &event)
// {
//     MsgJoyRumble.type       = 1;
//     MsgJoyRumble.id         = 1;
//     MsgJoyRumble.intensity  = 0.0;  
// }

void Robot::ClearPath(Path_t &path)
{
    while(!path.x.empty())
    {
        path.x.pop();
        path.y.pop();
        path.theta.pop();
    }
}

Robot::Pose_t Robot::PurePursuit(Pose_t robot_pose, Path_t &path, float offset)
{

    Pose_t target_pose;
    target_pose = robot_pose;

    float distance = 0;
    static float epsilon = 0.1;
    int pathLeft = path.x.size();

    target_pose.x = path.x.front();
    target_pose.y = path.y.front();
    target_pose.theta = path.theta.front();
    distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));
    
    if(pathLeft > 1)
    {    
        while(distance < offset && pathLeft > 1)
        {
            path.x.pop(); target_pose.x = path.x.front();
            path.y.pop(); target_pose.y = path.y.front();  
            path.theta.pop(); target_pose.theta = path.theta.front();
            distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));
        }
    }
    else
    {
        target_pose.x = path.x.front();
        target_pose.y = path.y.front();
        target_pose.theta = path.theta.front();
        // ClearPath(path);
    }

    // Debug
    std::cout << "path left = " << pathLeft << std::endl;

    return target_pose;
}

Robot::Pose_t Robot::PointToPointPID(Pose_t robot_pose, Pose_t target_pose, float maxSpeed)
{
    Pose_t robot_vel;
    robot_vel.x = robot_vel.y = robot_vel.theta = 0;

    float output[3] = {0, 0, 0};

    float proportional[3] = {0, 0, 0};
    float integral[3] = {0, 0, 0};
    float derivative [3]= {0, 0, 0};

    float error[3] = {0, 0, 0};
    static float prevError[3] = {0, 0, 0};
    static float sumError[3] = {0, 0, 0};

    float kp[3] = {40, 40, 40};
    float ki[3] = {0, 0, 0};
    float kd[3] = {0, 0, 0};

    error[0] = target_pose.x - robot_pose.x;
    error[1] = target_pose.y - robot_pose.y;
    error[2] = target_pose.theta - robot_pose.theta; 

    if(abs(error[2]) >= MATH_PI){
        if(error[2] > 0){
            error[2] = error[2] - 2*MATH_PI;
        }
        else{
            error[2] = error[2] + 2*MATH_PI;
        }
    }

    for(int i=0 ; i<=2 ; i++){
        sumError[i] += error[i];

        proportional[i] = kp[i] * error[i];
        integral[i] = ki[i] * sumError[i];
        derivative[i] = kd[i] * (error[i] - prevError[i]);

        prevError[i] = error[i];

        output[i] = proportional[i] + integral[i] + derivative[i];
    }

    // // Speed Limiter and Normalizer
    // if(abs(output[0]) >= maxSpeed || abs(output[1]) >= maxSpeed || abs(output[2]) >= maxSpeed){
    //     float max = 0;
    //     for(int i=0 ; i<=2 ; i++){
    //         if(abs(output[i]) > max){
    //             max = abs(output[i]);
    //         }
    //     }
    //     for(int i=0 ; i<=2 ; i++){
    //         output[i] = output[i] * (maxSpeed / max);
    //     }
    // }

    // Speed Limiter Only
    for(int i=0 ; i<=2 ; i++){
        if(output[i] >= maxSpeed){
            output[i] = maxSpeed;
        }
        else if(output[i] <= -maxSpeed){
            output[i] = -maxSpeed;
        }
    }

    robot_vel.x = output[0];
    robot_vel.y = output[1];
    robot_vel.theta = output[2];

    return robot_vel;
}

