#include <ros/ros.h>
#include "control_layout.h"

Robot::Robot(): RosRate(100)
{
    // Initialize
    ROS_INFO("Robot Main Controller");
    
    // Subscriber & Publisher
    Sub_Joy             = Nh.subscribe("/joy", 18, &Robot::Joy_Callback, this);
    Sub_Joy_Battery     = Nh.subscribe("/battery", 10, &Robot::Joy_Battery_Callback, this);
    Sub_Odom            = Nh.subscribe("/odom", 10, &Robot::Odom_Callback, this);
    Sub_Pose            = Nh.subscribe("/amcl_pose", 10, &Robot::Pose_Callback, this);
    Sub_Path            = Nh.subscribe("/path", 10, &Robot::Path_Callback, this);

    Pub_Vel             = Nh.advertise<main_controller::ControllerData>("robot/cmd_vel", 10);
    Pub_Joy_Feedback    = Nh.advertise<sensor_msgs::JoyFeedbackArray>("/set_feedback", 10);

    // Initialize Speed Variable
    for (int i = 0; i <=2 ; i++){
        vel_msg.data.push_back(0);
    }

    // Initialize Robot Pose
    Robot_Pose.x = Robot_Pose.y = Robot_Pose.z = 0;

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

            // Trigger to reset path
            if (Controller.Buttons[circle] == 0 && Controller.prev_button[circle] == 1)
            {
                // Load Path from CSV File
                std::string filePath = ros::package::getPath("main_controller") + "/data/path_1.csv";
                ClearPath(Robot_Path);
                ReadPath(filePath, Robot_Path);
                int pathSize = Robot_Path.x.size();
                std::cout << "Read waypoints completed" << ", size of path = " << pathSize << std::endl;
            }
            Controller.prev_button[circle] = Controller.Buttons[circle];

            // Pure Pursuit Mode //
            Pose_t targetPose;
            Pose_t speedRobot;

            // THIS targetPose FORCED TO GO TO ORIGIN (0,0) IF THERE'S NO PATH EXIST
            targetPose = PurePursuit(Robot_Pose, Robot_Path, 0.5);
            speedRobot = PointToPointPID(Robot_Pose, targetPose, 30);

            RobotSpeed[0] = (int) speedRobot.x;
            RobotSpeed[1] = (int) speedRobot.y;
            RobotSpeed[2] = (int) speedRobot.z;

            // Set Robot Speed to Zero (CHECK AGAIN FOR PURE PURSUIT IMPLEMENTATION)
            // for(int i = 0; i<=2; i++)
            // {
            //     RobotSpeed[i] = 0;
            // }
        }

        // Go to Manual Control Mode
        else
        {
            ClearPath(Robot_Path);
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

void Robot::Odom_Callback(const nav_msgs::OdometryConstPtr &msg){

    Robot_Pose_2.x = msg->pose.pose.position.x;
    Robot_Pose_2.y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;

    m.getRPY(roll, pitch, yaw);
    
    Robot_Pose_2.z = yaw;

}

void Robot::ReadPath(std::string fileName, Path_t &path){
    std::ifstream file(fileName);
    std::string line;

    if(file.is_open()){
        // Read and discard the first line
        getline(file, line);

        while(getline(file, line)){
            // Add temp point variable

            // Create a stringstream to read the line
            std::stringstream ss(line);
            std::string token;

            // Read each comma-separated value from the line
            getline(ss, token, ',');
            path.x.push(stof(token));

            getline(ss, token, ',');
            path.y.push(stof(token));

            getline(ss, token, ',');
            path.z.push(stof(token));
        }
    }
}

void Robot::ClearPath(Path_t &path){
    path.x.empty();
    path.y.empty();
    path.z.empty();
}

Robot::Pose_t Robot::PurePursuit(Pose_t robotPose, Path_t &path, float offset)
{

    Pose_t targetPose;
    targetPose = robotPose;

    float distance = 0;
    static float epsilon = 0.1;
    int pathLeft = path.x.size();

    targetPose.x = path.x.front();
    targetPose.y = path.y.front();
    targetPose.z = path.z.front();
    distance = sqrt(pow((targetPose.x - robotPose.x), 2) + pow((targetPose.y - robotPose.y), 2));
    
    if(pathLeft > 1){    
        while(distance < offset && pathLeft > 1){
            path.x.pop(); targetPose.x = path.x.front();
            path.y.pop(); targetPose.y = path.y.front();  
            path.z.pop(); targetPose.z = path.z.front();
            distance = sqrt(pow((targetPose.x - robotPose.x), 2) + pow((targetPose.y - robotPose.y), 2));
        }
    }
    else{
        targetPose.x = path.x.front();
        targetPose.y = path.y.front();
        targetPose.z = path.z.front();
    }

    // Debug
    std::cout << "path left = " << pathLeft << std::endl;
    std::cout << "robotPose.x = " << robotPose.x << " robotPose.y = " << robotPose.y << " robotPose.theta = " << robotPose.z << std::endl;
    std::cout << "targetPose.x = " << targetPose.x << " targetPose.y = " << targetPose.y << " targetPose.theta = " << targetPose.z << std::endl;
    std::cout << "distance = " << distance << std::endl;

    return targetPose;
}

Robot::Pose_t Robot::PointToPointPID(Pose_t robotPose, Pose_t targetPose, float maxSpeed)
{
    Pose_t speedRobot;
    speedRobot.x = speedRobot.y = speedRobot.z = 0;

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

    error[0] = targetPose.x - robotPose.x;
    error[1] = targetPose.y - robotPose.y;
    error[2] = targetPose.z - robotPose.z; 

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

    // Speed Limiter and Normalizer
    if(abs(output[0]) >= maxSpeed || abs(output[1]) >= maxSpeed || abs(output[2]) >= maxSpeed){
        float max = 0;
        for(int i=0 ; i<=2 ; i++){
            if(abs(output[i]) > max){
                max = abs(output[i]);
            }
        }
        for(int i=0 ; i<=2 ; i++){
            output[i] = output[i] * (maxSpeed / max);
        }
    }

    // Speed Limiter Only
    for(int i=0 ; i<=2 ; i++){
        if(output[i] >= maxSpeed){
            output[i] = maxSpeed;
        }
        else if(output[i] <= -maxSpeed){
            output[i] = -maxSpeed;
        }
    }

    speedRobot.x = output[0];
    speedRobot.y = output[1];
    speedRobot.z = output[2];

    return speedRobot;
}


void Robot::Path_Callback (const nav_msgs::Path::ConstPtr &path_msg)
{
    for(int i = 0; i < path_msg->poses.size(); ++i)
    {
        Robot_Path.x.push(path_msg->poses[i].pose.position.x);
        Robot_Path.y.push(path_msg->poses[i].pose.position.y);
        Robot_Path.z.push(path_msg->poses[i].pose.position.z);       
    }
}
void Robot::Pose_Callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    Robot_Pose.x = pose_msg->pose.pose.position.x;
    Robot_Pose.y = pose_msg->pose.pose.position.y;
    Robot_Pose.z = pose_msg->pose.pose.position.z;    
}
