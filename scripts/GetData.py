#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
import time
from tf import transformations
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

data_odom = (0, 0, 0)
data_amcl = (0, 0, 0)

def AMCLCallback(data):
    global data_amcl 
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y
    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    pose_theta = transformations.euler_from_quaternion(quaternion)[2]
    data_amcl = (pose_x, pose_y, pose_theta)
    
def OdomCallback(data):
    global data_odom
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y
    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    pose_theta = transformations.euler_from_quaternion(quaternion)[2]
    data_odom = (pose_x, pose_y, pose_theta)

# Kalau Mau Nambah Data Bikin Function
# def fungsiCallback():
#     global variabel
#     .
#     .
#     .
#     data_variabel = (..., ..., ..., ..., ...)

    
def listener():
    rospy.init_node('listener', anonymous=True)
    rate = rospy.Rate(10)
    time_start = rospy.Time.now().to_sec()
    rospy.Subscriber("/odom", Odometry, OdomCallback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, AMCLCallback)
    data = pd.DataFrame({"Time":[0],
                         "AMCL":[(0, 0, 0)],
                         "Odom":[(0, 0, 0)]})       # Tambah satu data lagi "Variabel:[(..., ..., ..., ..., ...)]"
    
    try:
        while not rospy.is_shutdown():
            time_now = rospy.Time.now().to_sec() - time_start
            new_data = {"Time": time_now,
                        "AMCL": data_amcl,
                        "Odom" : data_odom}         # Tambah satu data "Variabel: data_variabel"
            
            data.loc[len(data)] = new_data
            time.sleep(0.1)

    except KeyboardInterrupt:
        print ("Mboh")
    
    data.to_csv("tes.csv")


if __name__ == '__main__':
    listener()