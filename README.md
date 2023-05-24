# Main Controller
Main Control Layout for Service Robot ASR-ITS

## Usage
```bash
# Push PS Button on Controller
# Use this command if Controller not connected
cd
./ds4_autoconnect.sh

# Connect STM32 Cable to PC
# Connect RPLidar Data+Power Cable to PC
cd 
./chmodbus01.sh

# Connect ZED Camera Cable to PC
roslaunch zed_wrapper zed2i.launch

# Launch Main File
cd slam_ws
source devel/setup.bash
roslaunch main_controller asr_its.launch

# Use Waypoint Generator if Needed
rosrun path_planning waypoint.py
```

## Dependencies
- ds4_driver https://github.com/naoki-mizuno/ds4_driver
- zed_wrapper https://github.com/stereolabs/zed-ros-wrapper
- rplidar_ros https://github.com/Slamtec/rplidar_ros
