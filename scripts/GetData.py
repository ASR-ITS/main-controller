#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
import time
import math
import os
from tf import transformations
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

data_odom = (0, 0, 0, 0, 0, 0, 0, 0)
data_amcl = (0, 0, 0, 0, 0, 0, 0, 0)
data_goal = (0, 0, 0, 0)

def AMCLCallback(data):
    global data_amcl 
    iterasi = data.header.seq
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y
    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    pose_theta = transformations.euler_from_quaternion(quaternion)[2]
    cov_xx = data.pose.covariance[0]
    cov_xy = data.pose.covariance[1]
    cov_yy = data.pose.covariance[7]
    cov_thth = data.pose.covariance[35]
    data_amcl = (iterasi, pose_x, pose_y, pose_theta, cov_xx, cov_xy, cov_yy, cov_thth)
    
def OdomCallback(data):
    global data_odom
    iterasi = data.header.seq
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y
    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    pose_theta = transformations.euler_from_quaternion(quaternion)[2]
    cov_xx = data.pose.covariance[0]
    cov_xy = data.pose.covariance[1]
    cov_yy = data.pose.covariance[7]
    cov_thth = data.pose.covariance[35]
    data_odom = (iterasi, pose_x, pose_y, pose_theta, cov_xx, cov_xy, cov_yy, cov_thth)

def GoalCallback(data):
    global data_goal
    iterasi = data.header.seq
    pose_x = data.pose.position.x
    pose_y = data.pose.position.y
    quaternion = (
    data.pose.orientation.x,
    data.pose.orientation.y,
    data.pose.orientation.z,
    data.pose.orientation.w)
    pose_theta = transformations.euler_from_quaternion(quaternion)[2]
    data_goal = (iterasi, pose_x, pose_y, pose_theta)
    

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
    rospy.Subscriber("/odom_icp", Odometry, OdomCallback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, AMCLCallback)
    rospy.Subscriber("/goal", PoseStamped, GoalCallback)
    nama_file = rospy.get_param("/get_data/nama_file")
    directory = "/home/asr-its/Rayhansss/" + nama_file + ".csv"
    data = pd.DataFrame({"Time":[0],
                         "AMCL":[(0, 0, 0, 0, 0, 0, 0, 0)],
                         "Odom":[(0, 0, 0, 0, 0, 0, 0, 0)],
                         "Goal":[(0, 0, 0, 0)]})       # Tambah satu data lagi "Variabel:[(..., ..., ..., ..., ...)]"
    
    try:
        while not rospy.is_shutdown():
            time_now = rospy.Time.now().to_sec() - time_start
            new_data = {"Time": time_now,
                        "AMCL": data_amcl,
                        "Odom": data_odom,
                        "Goal": data_goal}         # Tambah satu data "Variabel: data_variabel"
            
            data.loc[len(data)] = new_data
            time.sleep(0.1)

    except KeyboardInterrupt:
        print ("Mboh")

    
    
    data.to_csv(directory)


if __name__ == '__main__':
    listener()