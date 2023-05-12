#include <ros/ros.h>
#include "control_layout.h"

Robot::Robot(): RosRate(100)
{
    // Initialize
    ROS_INFO("Robot Main Controller");
    
    // Subscriber & Publisher
    SubJoy          = Nh.subscribe("/joy", 18, &Robot::JoyCallback, this);
    SubJoyBattery   = Nh.subscribe("/battery", 10, &Robot::JoyBatteryCallback, this);
    SubOdom         = Nh.subscribe("/odom", 10, &Robot::OdomCallback, this);
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

    // Read path from csv file
    std::string filePath = ros::package::getPath("main_controller") + "/data/path_1.csv";
    std::cout << "Waypoints file path: " << filePath << std::endl;
    ReadPath(filePath, targetPath);
    std::cout << "Read waypoints completed" << ", size of path = " << targetPath.size() << std::endl;

    while(ros::ok()){
        // Set Status Control using Triangle Button
        if (Controller.Buttons[triangle] == 0 && Controller.prev_button[triangle] == 1)
        {
            StatusControl ^= 1;
            MsgSpeed.StatusControl = StatusControl;
        }
        Controller.prev_button[triangle] = Controller.Buttons[triangle];

        // Print Robot Speed to Screen
        // std::cout << "Speed[0] : " << RobotSpeed[0] << " Speed[1] : " << RobotSpeed[1] << " Speed[2] : " << RobotSpeed[2] << " Status : " << MsgSpeed.StatusControl << " Joystick Battery : " << JoyBatt*100 << "%" << std::endl;

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

            // Pure Pursuit Mode //

            geometry_msgs::Pose2D targetPose;
            geometry_msgs::Twist speedRobot;

            targetPose = PurePursuit(robotPose, targetPath, 0.5);
            speedRobot = PointToPointPID(robotPose, targetPose);

            RobotSpeed[0] = (int) 25 * speedRobot.linear.x;
            RobotSpeed[1] = (int) 25 * speedRobot.linear.y;
            RobotSpeed[2] = (int) 25 * speedRobot.angular.z;

            std::cout << "Speed[0] : " << RobotSpeed[0] << " Speed[1] : " << RobotSpeed[1] << " Speed[2] : " << RobotSpeed[2] << " Status : " << MsgSpeed.StatusControl << " Joystick Battery : " << JoyBatt*100 << "%" << std::endl;

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

void Robot::OdomCallback(const nav_msgs::OdometryConstPtr &msg){

    robotPose.x = msg->pose.pose.position.x;
    robotPose.y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;

    m.getRPY(roll, pitch, yaw);
    
    robotPose.theta = yaw;

}

void Robot::ReadPath(std::string fileName, std::queue<point_t> &path){
    std::ifstream file(fileName);
    std::string line;

    if(file.is_open()){
        // Read and discard the first line
        getline(file, line);

        while(getline(file, line)){
            // Add temp point variable
            point_t tempPoint;

            // Create a stringstream to read the line
            std::stringstream ss(line);
            std::string token;

            // Read each comma-separated value from the line
            getline(ss, token, ',');
            tempPoint.x = stof(token);

            getline(ss, token, ',');
            tempPoint.y = stof(token);

            getline(ss, token, ',');
            tempPoint.theta = stof(token);

            // Push into robot's path variable
            path.push(tempPoint);
        }
    }
}

geometry_msgs::Pose2D Robot::PurePursuit(geometry_msgs::Pose2D robotPose, std::queue<point_t> &path, float offset)
{
    
    int length = path.size();
    float distance = 0;
    static float epsilon = 0.1;

    geometry_msgs::Pose2D out;
    out.x = out.y = out.theta = 0;
    point_t targetPoint;

    targetPoint = path.front();
    distance = sqrt(pow((targetPoint.x - robotPose.x), 2) + pow((targetPoint.y - robotPose.y), 2));
    
    if(path.size() > 1){    
        while(distance < offset && path.size() > 1){
            distance = sqrt(pow((targetPoint.x - robotPose.x), 2) + pow((targetPoint.y - robotPose.y), 2));
            std::cout << "size = " << path.size() << " dist = " << distance << std::endl;
            path.pop();
            targetPoint = path.front();
        }
    }
    else{
        targetPoint = path.front();
    }

    out.x = targetPoint.x;
    out.y = targetPoint.y;
    out.theta = targetPoint.theta;

    // Debug
    std::cout << "path left = " << length << std::endl;
    std::cout << "robotPose.x = " << robotPose.x << " robotPose.y = " << robotPose.y << " robotPose.theta = " << robotPose.theta << std::endl;
    std::cout << "targetPoint.x = " << targetPoint.x << " targetPoint.y = " << targetPoint.y << " targetPoint.theta = " << targetPoint.theta << std::endl;
    std::cout << "distance = " << distance << std::endl;

    return out;
}

geometry_msgs::Twist Robot::PointToPointPID(geometry_msgs::Pose2D robotPose, geometry_msgs::Pose2D targetPose)
{
    geometry_msgs::Twist out;
    out.linear.x = 0;
    out.linear.y = 0;
    out.angular.z = 0;

    float speedRobot[3] = {0, 0, 0};
    float maxSpeed = 25;

    float proportional[3] = {0, 0, 0};
    float integral[3] = {0, 0, 0};
    float derivative [3]= {0, 0, 0};
    float output[3] = {0, 0, 0};

    float error[3] = {0, 0, 0};
    static float prevError[3] = {0, 0, 0};
    static float sumError[3] = {0, 0, 0};

    float kp[3] = {1, 1, 1};
    float ki[3] = {0, 0, 0};
    float kd[3] = {0, 0, 0};

    error[0] = targetPose.x - robotPose.x;
    error[1] = targetPose.y - robotPose.y;
    error[2] = targetPose.theta - robotPose.theta; 

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

        speedRobot[i] = proportional[i] + integral[i] + derivative[i];
    }

    // Speed Limiter and Normalizer
    if(abs(speedRobot[0]) >= maxSpeed || abs(speedRobot[1]) >= maxSpeed || abs(speedRobot[2]) >= maxSpeed){
        float max = 0;
        for(int i=0 ; i<=2 ; i++){
            if(abs(speedRobot[i]) > max){
                max = abs(speedRobot[i]);
            }
        }
        for(int i=0 ; i<=2 ; i++){
            speedRobot[i] = speedRobot[i] * (maxSpeed / max);
        }
    }

    // Speed Limiter Only
    for(int i=0 ; i<=2 ; i++){
        if(speedRobot[i] >= maxSpeed){
            speedRobot[i] = maxSpeed;
        }
        else if(speedRobot[i] <= -maxSpeed){
            speedRobot[i] = -maxSpeed;
        }
    }

    out.linear.x = speedRobot[0];
    out.linear.y = speedRobot[1];
    out.angular.z = speedRobot[2];

    return out;
}



