#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <termios.h>
#include <unistd.h>

#define MATH_PI 3.1415926535897932

int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dummy_robot");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10);

    geometry_msgs::PoseWithCovarianceStamped cmd_odom;

    cmd_odom.pose.pose.position.x = 0.0;
    cmd_odom.pose.pose.position.y = 0.0;
    cmd_odom.pose.pose.position.z = 0.0;

    odom_pub.publish(cmd_odom);

    while (ros::ok()) {
        int c = getch();
        switch (c) {
            case 'w':
                cmd_odom.pose.pose.position.y += 0.05;
                printf("w");
                break;
            case 's':
                cmd_odom.pose.pose.position.y -= 0.05;
                printf("s");
                break;
            case 'a':
                cmd_odom.pose.pose.position.x -= 0.05;
                printf("a");
                break;
            case 'd':
                cmd_odom.pose.pose.position.x += 0.05;
                printf("d");
                break;
            case 'e':
                cmd_odom.pose.pose.position.z += 0.05;
                printf("e");
                break;
            case 'q':
                cmd_odom.pose.pose.position.z -= 0.05;
                printf("q");
                break;
            // default:
            //     cmd_vel.linear.x = 0.0;
            //     cmd_vel.angular.z = 0.0;
        }
        if (cmd_odom.pose.pose.position.z > 180)
        {
            cmd_odom.pose.pose.position.z = -180;
        }
        else if (cmd_odom.pose.pose.position.z < -180)
        {
            cmd_odom.pose.pose.position.z = 180;
        }

        odom_pub.publish(cmd_odom);

        ros::spinOnce();
    }

    return 0;
}