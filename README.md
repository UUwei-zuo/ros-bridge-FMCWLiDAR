# CARLA-ROS Bridge with FMCW LiDAR Support

 - The [ros-bridge](https://github.com/carla-simulator/ros-bridge) is a package that enables two-way communication between [ROS simulator](https://www.ros.org) and [CARLA simulator](https://github.com/carla-simulator/carla).
 - FMCW (Doppler) LiDAR is a variant of traditional LiDAR that provides 4D point cloud with [Doppler velocity](https://en.wikipedia.org/wiki/Doppler_effect).
 - In this fork, FMCW LiDAR support is added to improve adaptibility. 

![rviz setup](./docs/images/ad_demo.png "AD Demo")

## Features

- 🎯Facilitate CARLA-ROS bridge with FMCW LiDAR support.

## Installation 

- **🧐It is suggested that you read official documents for [ROS bridge installation](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/#before-you-begin) first.** 
- 1️⃣To begin with, you need to clone and compile [carla-aeva](https://github.com/aevainc/carla-aeva) which contains FMCW LiDAR plugin.
- 2️⃣Then, you can start to establish your own CARLA-ROS project with this enhanced bridge.



The following example shows the installation on [ROS Neotic](https://wiki.ros.org/noetic/Installation/Ubuntu) and Ubuntu 20.04:

Clone this bridge by 
```
mkdir -p ~/carla-ros-bridge/catkin_ws/src
cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/UUwei-zuo/ros-bridge-FMCWLiDAR.git catkin_ws/src/ros-bridge
source /opt/ros/noetic/setup.bash 
```
Install ROS dependencies and build the bridge by
```
cd catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r
catkin build 
```
Congratulations!!

😺Click the **Star** if you find this repo helpful :)
