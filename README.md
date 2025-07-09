
# QuadrupedA1Controller

A collection of ROS2 Packages to handle the simulation of the Unitree A1 Quadruped robot using ROS2 Jazzy and Gazebo Sim Harmonic. This repository includes everthing necessary (IMU-/ Contact-Sensors, ROS2 Interface, ...) to simulate and control the robot using joint space commands (position or torque). High level control (like locomotion) is currently under development.
## Installation


Important Packages:
- numpy, scipy, matplotlib
- a recent version of Pygame (for Kinematics Demo GUI / Locomotion Controller, but "/pose" and "/cmdvel" topics can also be published via commandline or RQT!)
- ROS2 Jazzy
- Gazebo Sim Harmonic (older distributions of Gazebo Sim do not fully support e.g. the Feet Contact Sensors)

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


# Usage:

### Wholebody Kinematics Demo:

This script uses the Gazebo Sim Plugins for PID Control and our inverse kinematics model to showcase the translation/rotation of the robot's body in all 6 movement axes. 
We set up a small Pygame window to use a joystick GUI for easy and intuitive input.


<img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExNDgwZDV5MG5sa2V5cGNrNDV1YXhmMHJscjZkNHBpd3RzMHY1Znp1MiZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/Zwn2wgBUeAppXXqY6Q/giphy.gif" alt="Alt Text" width="450">


The Gazebo environment with all necessary control topics can be launched using:
```
ros2 launch ros_gz_a1_bringup a1_gazebo_sim.launch.py
```

To use the wholebody kinematics controller run the following two nodes:
```
ros2 run ros_gz_a1_controller joint_state_publisher
```
and
```
ros2 run ros_gz_a1_ui pose_pub_gui
```

### Locomotion Controller:
This is our implementation of a trotting gait using adaptive bezier curves for trajectory generation. This controller enables the robot to move forward (/backward) and rotate around the robot's z-axis (either on the spot or when moving). Currently the robot is capable of reaching every point on a planar world.

![Alt Text](https://i.giphy.com/media/v1.Y2lkPTc5MGI3NjExbHVocnRzNWRqcXlsNzMyamZnMHk1Yzhna2diaWdud3gxYmszOTJoeCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/m4xsEcm3qinwZwdHMF/giphy.gif)

To use the locomotion controller run the following commands:
```
ros2 launch ros_gz_a1_bringup a1_gazebo_sim.launch.py use_force_ctrl:=True
```
(Press start in Gazebo Sim before running the low_level_controller to let the robot lay on the ground)
```
ros2 run ros_gz_a1_controller low_level_controller
```

```
ros2 run ros_gz_a1_controller a1_controller
```

```
ros2 run ros_gz_a1_ui cmdvel_pub_gui
```
