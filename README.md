# Main Controller
Main Control Layout for Service Robot ASR-ITS

## Usage
```bash
# Push PS Button on Controller
cd
./ds4_autoconnect.sh

# Connect STM32 Cable to PC
cd slam_ws
source devel/setup.bash
sudo chmod 777 /dev/ttyUSB0

# Connect RPLidar Data+Power Cable to PC
sudo chmod 777 /dev/ttyUSB1

# Connect ZED Camera Cable to PC
roslaunch zed_wrapper zed2i.launch

# Launch Main File
roslaunch main_controller asr_its.launch
```

## Dependencies
- ds4_driver https://github.com/naoki-mizuno/ds4_driver
- zed_wrapper https://github.com/stereolabs/zed-ros-wrapper
- rplidar_ros https://github.com/Slamtec/rplidar_ros
