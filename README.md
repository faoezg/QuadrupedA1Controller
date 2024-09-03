
# QuadrupedA1Controller

A collection of ROS2 Packages to handle the simulation of the Unitree A1 Quadruped robot using ROS2 Jazzy and Gazebo Sim Harmonic. This repository includes everthing necessary (IMU-/ Contact-Sensors, ROS2 Interface, ...) to simulate and control the robot using joint space commands. High level control (like locomotion) is currently under development.
## Installation


Important Packages:
- a recent version of Pygame

Installation:
- in /ros2_ws/src
- git clone this repository
- build the packages using colcon 

## Overview:
The ROS Package a1_controller consists of the necessary scripts to control the robot in the new Gazebo Sim.
Our project structure is based on the [ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template).
- ros_gz_a1_bringup: launch/config files to start the simulation and ros2 bridge
- ros_gz_a1_controller: kinematics library and ros nodes to control the robot
- ros_gz_a1_description: Unitree A1 .sdf file and meshes
- ros_gz_a1_gazebo: World description files for Gazebo Sim

![Alt Text](https://i.imgur.com/f0Jjd32.png)

The Above image is a visualization of the foot workspace and redundant positions (those with multiple viable joint configurations colored in blue) in RViz using Pointclouds.

## Usage:
This script uses the Gazebo Sim Plugins for PID Control and our inverse kinematics model to showcase the translation/rotation of the robot's body in all 6 movement axes. 
We set up a small Pygame window to use a joystick GUI for easy and intuitive input.


![Alt Text](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExNDgwZDV5MG5sa2V5cGNrNDV1YXhmMHJscjZkNHBpd3RzMHY1Znp1MiZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/Zwn2wgBUeAppXXqY6Q/giphy.gif)

The Gazebo environment and the kinematics demo can be launched using:
```
ros2 launch ros_gz_a1_bringup a1_gazebo_sim.launch.py
```
then
```
ros2 run ros_gz_a1_controller joint_state_publisher
```
and
```
ros2 run ros_gz_a1_controller pose_pub_gui
```

