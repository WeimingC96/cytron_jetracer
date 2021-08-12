# Getting Started
## Cytron jetracer 
Follow this tutorial page: https://tutorial.cytron.io/?p=38120 to install all the libraries that are needed.

Then clone this repository.
- `cd ~/catkin_ws/src`
- `git clone https://github.com/WeimingC96/cytron_jetracer.git`
- `cd cytron_jetracer/scripts`
- `chmod +x camera.py camera_subscriber.py joint_state.py racecar.py teleop.py teleop_gamepad.py`
- `cd ~/catkin_ws`
- `catkin_make`

## YDLidar ROS Driver
YDLidar ROS Driver depends on the YDLidar-SDK library, so you must first install the YDLidar-SDK library. It's better to have a new workspace for this library.
- `cd ~/lidar_sdk_ws/src`
- `git clone https://github.com/YDLIDAR/YDLidar-SDK.git'
- `cd YDLidar-SDK/build`
- `cmake ..`
- `make`
- `sudo make install`

Back to your previous workspace and install the ROS Driver.
- `cd ~/catkin_ws/src`
- `git clone https://github.com/WeimingC96/ydlidar_ros_driver.git`
- `cd ..`
- `catkin_make`

Add a device alias /dev/ydlidar to the X4 serial port.
- `cd catkin_ws/src/ydlidar/startup`
- `sudo chmod +x initenv.sh`
- `sudo sh initenv.sh`

RVIZ view scan results
- `source ~/catkin_ws/devel/setup.bash`
- `roslaunch ydlidar_ros_driver X4.launch`
- `roslaunch ydlidar_ros_driver lidar_view.launch`

The scanning data seen by running the launch file is displayed by default with 360-degree data. To modify the display range, you need to modify the configuration parameters in the launch file.

For more information about the file contents, please refer to: https://github.com/WeimingC96/ydlidar_ros_driver.git

## rf2o_laser_odometry
Estimation of 2D odometry based on planar laser scans.
- `cd catkin_ws/src`
- `git clone https://github.com/WeimingC96/rf2o_laser_odometry.git`
- `cd ..`
- `catkin_make`

## Gmapping
- `cd catkin_ws/src`
- `git clone https://github.com/WeimingC96/slam_gmapping.git`
- `cd slam_gmapping`
- `git clone https://github.com/ros-perception/openslam_gmapping.git`
- `cd ..`
- `catkin_make`

## Onboard
The launch file `jetracer_onboard.launch` contains
- A node coverting camera images to ROS message
- A node subscribing remote control inputs from the controller
- A node starting the YDLidar

Useful bash aliases:
- alias onboard="source ~/Weiming_ws/devel/setup/bash; roslaunch cytron_jetracer jetracer_onboard.launch"

## On you laptop
The launch file `jetracer_remote.launch` contains
- A node subscribing camera image for display
- A node converting steering angles / throttle from remote controller to Jetracer inputs between (-1, 1)
- A localization node for fusing scan and odometry

Useful bash aliases:
- alias jetracer_remote="source ~/catkin_ws/devel/setup.bash; roslaunch cytron_jetracer jetracer_remote.launch"
- alias mapping="roslaunch gmapping slam_gmapping_pr2.launch"
- alias jetson="ssh jetson@192.168.0.142"

## Running the Jetracer
### Note: The following steps should all be done on your laptop
Turn on the Jetracer. Open a terminal:
- `roscore`

Open a new terminal:
- `jetson`
- `onboard`

Open a new terminal:
- `jetracer_remote`
- `mapping`

