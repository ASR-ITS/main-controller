#include <ros/ros.h>
#include "control_layout.h"

Robot::Robot(): RosRate(100)
{
    // Initialize
    ROS_INFO("Robot Main Controller");
    
    // Subscriber & Publisher
    Sub_Joy            = Nh.subscribe("/joy", 18, &Robot::Joy_Callback, this);

    Sub_Path           = Nh.subscribe("/path", 10, &Robot::Path_Callback, this);
    Sub_Pose           = Nh.subscribe("/amcl_pose", 10, &Robot::Pose_Callback, this);

    Sub_Crashed        = Nh.subscribe("/crashed", 1, &Robot::Crashed_Status_Callback, this);
    Sub_Obstacle       = Nh.subscribe("/obstacle_detected", 1, &Robot::Obstacle_Status_Callback, this);
    Sub_Obs_Vel        = Nh.subscribe("/velocity_obstacle/opt_vel", 10, &Robot::Obstacle_Vel_Callback, this);

    Pub_Vel            = Nh.advertise<main_controller::ControllerData>("robot/cmd_vel", 10);
    Pub_Joy_Feedback   = Nh.advertise<sensor_msgs::JoyFeedbackArray>("/set_feedback", 10);
    Pub_Origin         = Nh.advertise<geometry_msgs::PoseStamped>("/goal", 1);
    Pub_Pure_Pursuit   = Nh.advertise<geometry_msgs::Twist>("/pure_pursuit_vel", 10);
    Pub_Local_Desired_Vel = Nh.advertise<geometry_msgs::Twist>("/main_controller/local_desired_vel", 10);

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
    int path_left;
    double distance_left;

    while(ros::ok())
    {   
        // Print Robot Speed (DEBUG)
        // std::cout << "x : " << robot_vel[0] << " y : " << robot_vel[1] << " Theta : " << robot_vel[2] << " Status : " << vel_msg.StatusControl << std::endl;
        // std::cout << "pose= x: " << robot_pose.x << " y: " << robot_pose.y << " theta: " << robot_pose.theta*(180/MATH_PI) << std::endl;

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

        // Set GUIDED/MANUAL Mode Using OPTIONS Button
        if (Controller.Buttons[OPTIONS] == 0 && Controller.prev_button[OPTIONS] == 1)
        {
            GuidedMode ^= 1;
            // Set Rumble Feedback
            rumble_status = 1;
            MsgJoyRumble.intensity  = 0.5;
            prev_time = ros::Time::now();
        }
        Controller.prev_button[OPTIONS] = Controller.Buttons[OPTIONS];

        // Set RTH Mode Using SHARE Button
        if (Controller.Buttons[SHARE] == 1 && Controller.prev_button[SHARE] == 0)
        {
            // Set Robot Mode to RTH
            // RTHMode = 1;
            // GuidedMode = 0;

            // Clear Current Path
            ClearPath(path);

            // Add Header Goal Message
            origin_msg.header.stamp = ros::Time::now();
            origin_msg.header.frame_id = "map";

            // Set Goal to Origin Position
            origin_msg.pose.position.x = 0.0;
            origin_msg.pose.position.y = 0.0;
            origin_msg.pose.position.z = 0.0;

            // Set Goal to Zero Degree Orientation
            origin_msg.pose.orientation.x = 0.0;
            origin_msg.pose.orientation.y = 0.0;
            origin_msg.pose.orientation.z = 0.0;
            origin_msg.pose.orientation.w = 1.0;

            // Publish Goal Message
            Pub_Origin.publish(origin_msg);

            // Set Rumble Feedback
            rumble_status = 1;
            MsgJoyRumble.intensity  = 0.5;
            prev_time = ros::Time::now();
        }
        Controller.prev_button[SHARE] = Controller.Buttons[SHARE];

        // Rumble Feedback Event
        if (rumble_status && ros::Time::now() - prev_time >= ros::Duration(0.57))
        {
            MsgJoyRumble.intensity  = 0.0;
            rumble_status = 0;
        }

        // Clear Path if Robot CRASHED
        if (prev_crashed == 0 && crashed_status == 1)
        {
            // DEBUG
            ClearPath(path);
            ROS_INFO("Robot stopped because path is closed. Recalculating path... ");
        }
        prev_crashed = crashed_status;

        // Go to Autonomous Mode
        if (GuidedMode)
        {
            // Go to AUTONOMOUS Mode with Indicator
            if (vel_msg.StatusControl)
            {
                // Set YELLOW Indicator for GUIDED Mode
                if(GuidedMode == 1)
                {
                    // Set LED Feedback
                    MsgJoyLED_R.intensity = 0.3;
                    MsgJoyLED_G.intensity = 0.3;
                    MsgJoyLED_B.intensity = 0.0;
                }
                // Set BLUE Indicator for GUIDED Mode
                // if(RTHMode == 1 && GuidedMode != 1)
                // {
                //     // Set LED Feedback
                //     MsgJoyLED_R.intensity = 0.0;
                //     MsgJoyLED_G.intensity = 0.0;
                //     MsgJoyLED_B.intensity = 0.5;
                // }

                // Prevent going to origin if there's no path
                if(path.x.size() <= 0)
                {
                    next_pose = robot_pose;
                }
                // Search for Closest Node
                else
                {
                    next_pose = PurePursuit(robot_pose, path, 0.1, obstacle_status);
                }

                // if(path_left <= 1)
                // {
                //     distance_left = sqrt(pow((next_pose.x - robot_pose.x), 2) + pow((next_pose.y - robot_pose.y), 2));
                //     if(distance_left <= 0.072)
                //     {

                //         ClearPath(path);
                //     }
                // }

                // else
                // {
                // Pure Pursuit PID
                pure_pursuit_vel = PointToPointPID(robot_pose, next_pose, 30);
                // LQR PID
                // pure_pursuit_vel = PointToPointLQR(robot_pose, next_pose, 20);

                // Push Pure Pursuit Velocity to Publisher Messages
                pure_pursuit_msg.linear.x  = (int) pure_pursuit_vel.x;
                pure_pursuit_msg.linear.y  = (int) pure_pursuit_vel.y;
                pure_pursuit_msg.angular.z = (int) pure_pursuit_vel.theta;

                // Convert Pure Pursuit Velocity to Local Velocity
                local_vel = Global_to_Local_Vel(robot_pose, pure_pursuit_vel);

                robot_vel[0] = local_vel.x;
                robot_vel[1] = local_vel.y;
                robot_vel[2] = local_vel.theta;

                local_desired_vel_msg.linear.x = local_vel.x * cos(MATH_PI/2) + local_vel.y * sin(MATH_PI/2);
                local_desired_vel_msg.linear.y = -1 * local_vel.x * sin(MATH_PI/2) + local_vel.y * cos(MATH_PI/2);
                local_desired_vel_msg.angular.z = local_vel.theta;

                // Obstacle Avoidance Control with WHITE Indicator
                if(obstacle_status)
                {
                    // Set LED Feedback
                    MsgJoyLED_R.intensity = 1.0;
                    MsgJoyLED_G.intensity = 1.0;
                    MsgJoyLED_B.intensity = 1.0;

                    robot_vel[0] = obstacle_avoider_vel.x * cos(MATH_PI/2) - obstacle_avoider_vel.y * sin(MATH_PI/2);
                    robot_vel[1] = obstacle_avoider_vel.x * sin(MATH_PI/2) + obstacle_avoider_vel.y * cos(MATH_PI/2);
                    robot_vel[2] = local_vel.theta;
                }
                // }
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
                robot_vel[0] = -1 * Controller.Axis[0] * 30;
                robot_vel[1] = Controller.Axis[1] * 30;
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
        Pub_Local_Desired_Vel.publish(local_desired_vel_msg);

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

void Robot::Path_Callback (const nav_msgs::Path::ConstPtr &path_msg)
{
    double  roll, pitch, yaw;

    for(int i = 0; i < path_msg->poses.size(); ++i)
    {
        // Convert Quaternion to Euler Yaw (Rad)
        tf::Quaternion q(
        path_msg->poses[i].pose.orientation.x,
        path_msg->poses[i].pose.orientation.y,
        path_msg->poses[i].pose.orientation.z,
        path_msg->poses[i].pose.orientation.w);
        tf::Matrix3x3 m(q);

        m.getRPY(roll, pitch, yaw);
        // Push Subscriber Topics to Path Array
        path.x.push(path_msg->poses[i].pose.position.x);
        path.y.push(path_msg->poses[i].pose.position.y);
        path.theta.push(yaw);    

    }
}

void Robot::Pose_Callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg)
{
    double  roll, pitch, yaw;

    // Convert Quaternion to Euler Yaw (Rad)
    tf::Quaternion q(
        pose_msg->pose.pose.orientation.x,
        pose_msg->pose.pose.orientation.y,
        pose_msg->pose.pose.orientation.z,
        pose_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
    // Push Subscriber Topics to Current Robot Pose Variable
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

void Robot::ClearPath(Path_t &path)
{
    while(!path.x.empty())
    {
        path.x.pop();
        path.y.pop();
        path.theta.pop();
    }
}

Robot::Pose_t Robot::PurePursuit(Pose_t robot_pose, Path_t &path, float offset, bool obstacle)
{

    Pose_t target_pose;
    target_pose = robot_pose;

    float distance = 0.0;
    float theta_error = 0.0;
    float dx, dy, dot_product;
    int pathLeft = path.x.size();

    // // Collision Avoidance Mode
    if(obstacle)
    {
        while(pathLeft > 1)
        {
            // Check for lookahead point
            target_pose.x = path.x.front();
            target_pose.y = path.y.front();
            target_pose.theta = path.theta.front();
            distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));

            // Check if the path point is behind the vehicle
            dx = target_pose.x - robot_pose.x;
            dy = target_pose.y - robot_pose.y;
            dot_product = dx * cos(robot_pose.theta) + dy * sin(robot_pose.theta);

            if(distance >= offset && dot_product > 0)
            {
                return target_pose;
            }
            else
            {
                path.x.pop(); path.y.pop(); path.theta.pop();
            }
        }
        // If Path Left is 1
        target_pose.x = path.x.front();
        target_pose.y = path.y.front();
        target_pose.theta = path.theta.front();

        // Check Distance and Theta Error for Last Target Point
        distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));
        theta_error = target_pose.theta - robot_pose.theta;
        
        if(abs(theta_error) >= MATH_PI)
        {
            if(theta_error > 0)
            {
                theta_error = theta_error - 2*MATH_PI;
            }
            else
            {
                theta_error = theta_error + 2*MATH_PI;
            }
        }

        if(distance <= 0.033 && theta_error <= MATH_PI/36 && theta_error >= -MATH_PI/36)
        {
            // Stop the Robot and Clear Path
            path.x.pop(); target_pose.x = robot_pose.x;
            path.y.pop(); target_pose.y = robot_pose.y;
            path.theta.pop(); target_pose.theta = robot_pose.theta;
            ROS_INFO("Path Finished!");
        }
    }

    // Normal Mode
    else
    {
        target_pose.x = path.x.front();
        target_pose.y = path.y.front();
        target_pose.theta = path.theta.front();
        distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));
        
        if(pathLeft > 1)
        {    
            while(distance < offset)
            {
                path.x.pop(); target_pose.x = path.x.front();
                path.y.pop(); target_pose.y = path.y.front();  
                path.theta.pop(); target_pose.theta = path.theta.front();
                if(path.x.size() <= 1)
                    break;
                distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));
            }
        }
        else
        {
            target_pose.x = path.x.front();
            target_pose.y = path.y.front();
            target_pose.theta = path.theta.front()/* - 180 * (MATH_PI/180)*/;

            // Check Distance and Theta Error for Last Target Point
            distance = sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2));
            theta_error = target_pose.theta - robot_pose.theta;
            
            if(abs(theta_error) >= MATH_PI)
            {
                if(theta_error > 0)
                {
                    theta_error = theta_error - 2*MATH_PI;
                }
                else
                {
                    theta_error = theta_error + 2*MATH_PI;
                }
            }

            if(distance <= 0.033 && theta_error <= MATH_PI/36 && theta_error >= -MATH_PI/36)
            {
            // Stop the Robot and Clear Path
            path.x.pop(); target_pose.x = robot_pose.x;
            path.y.pop(); target_pose.y = robot_pose.y;
            path.theta.pop(); target_pose.theta = robot_pose.theta;
            ROS_INFO("Path Finished!");
            }
        }
    }

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

    float kp[3] = {300, 300, 15};
    float ki[3] = {0, 0, 0};
    float kd[3] = {160, 160, 7};

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

    // Speed Limiter Only
    for(int i=0 ; i<=1 ; i++){
        if(output[i] >= maxSpeed){
            output[i] = maxSpeed;
        }
        else if(output[i] <= -maxSpeed){
            output[i] = -maxSpeed;
        }
    if(output[2] >= 20){
        output[2] = 20;
    }
    else if(output[2] <= -20){
        output[2] = -20;
    }
    }

    robot_vel.x = output[0];
    robot_vel.y = output[1];
    robot_vel.theta = output[2];

    // std::cout << "Pose theta = " << robot_pose.theta << " Target theta = " << target_pose.theta << std::endl;

    return robot_vel;

}

Robot::Pose_t Robot::PointToPointLQR(Pose_t robot_pose, Pose_t target_pose, float maxSpeed)
{
    const int maxIterations = 100;
    const double convergenceThreshold = 1e-6;

    float error[3] = {0, 0, 0};
    float output[3] = {0, 0, 0};
    float dt = 1;

    Pose_t robot_vel;
    robot_vel.x = robot_vel.y = robot_vel.theta = 0;


    error[0] = target_pose.x - robot_pose.x;
    error[1] = target_pose.y - robot_pose.y;
    error[2] = target_pose.theta - robot_pose.theta;

    // Nearest Angle
    if(abs(error[2]) >= MATH_PI){
        if(error[2] > 0){
            error[2] = error[2] - 2*MATH_PI;
        }
        else{
            error[2] = error[2] + 2*MATH_PI;
        }
    }

    // Define the system dynamics matrices A, B
    Eigen::MatrixXd A(3, 3);
    A << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    Eigen::MatrixXd B(3, 3);
    B << -cos(robot_pose.theta)*dt,  sin(robot_pose.theta)*dt,    0,
         -sin(robot_pose.theta)*dt, -cos(robot_pose.theta)*dt,    0,
                                 0,                         0, 1*dt;

    // Define the Q and R matrices
    // unchecked P2P diag(10, 10, 10)
    // unchecked PTrack diag(150, 150, 150)
    Eigen::MatrixXd Q(3, 3);
    Q << 20, 0, 0,
         0, 20, 0,
         0, 0, 10;

    Eigen::MatrixXd R(3, 3);
    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    // Solve the Algebraic Riccati Equation
    Eigen::MatrixXd P = Q;

    // Perform the iterative computation
    for (int i = 0; i < maxIterations; ++i) {
        Eigen::MatrixXd P_prev = P;
        P = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
        // std::cout << P << std::endl;

        // Check for convergence
        double normDiff = (P - P_prev).norm();
        if (normDiff < convergenceThreshold) {
            std::cout << "Convergence achieved after " << i+1 << " iterations." << std::endl;
            break;
        }
    }

    // Calculate the gain matrix K
    // Eigen::MatrixXd K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    Eigen::MatrixXd K = R.inverse() * B.transpose() * P;

    Eigen::Vector3d Error(error[0], error[1], error[2]);
    Eigen::Vector3d U = -K * Error;

    // Extract individual control inputs
    output[0] = U[0];
    output[1] = U[1];
    output[2] = U[2];

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
    robot_vel.theta = -output[2];

    // Display the optimal LQR Parameter
    std::cout << "Optimal control gain matrix K:" << std::endl;
    std::cout << K << std::endl;
    std::cout << "error E " << std::endl;
    std::cout << Error << std::endl;
    std::cout << "control U " << std::endl;
    std::cout << U << std::endl;
    // Display the control inputs
    std::cout << "Control inputs:" << std::endl;
    std::cout << "robot_vel.x: " << robot_vel.x << std::endl;
    std::cout << "robot_vel.y: " << robot_vel.y << std::endl;
    std::cout << "robot_vel.z: " << robot_vel.theta << std::endl;

    return robot_vel;

}

Robot::Pose_t Robot::Global_to_Local_Vel(Pose_t robot_pose, Pose_t global_vel)
{
    Pose_t local_vel;

    local_vel.x = global_vel.x * sin(robot_pose.theta) - global_vel.y * cos(robot_pose.theta);
    local_vel.y = global_vel.x * cos(robot_pose.theta) + global_vel.y * sin(robot_pose.theta);
    local_vel.theta = global_vel.theta;

    return local_vel;
}
