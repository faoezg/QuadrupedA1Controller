
# QuadrupedA1Controller

A collection of Python scripts to handle the kinematics of the Unitree A1 Quadruped robot
## Installation


Important Packages:
- unitree_ros: 
    https://github.com/unitreerobotics/unitree_ros.git

- unitree_ros_to_real:
    https://github.com/unitreerobotics/unitree_ros_to_real.git

- unitree_legged_sdk v3.3.4:
    https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/3.3.4
- a recent version of Pygame

Installation:
- in ~/catkin_ws/src/
- git clone the unitree_ros repository
- inside unitree_ros: copy contents of unitree_ros_to_real into the corresponding folder
- download unitree_legged_sdk v3.3.4, unzip and place the SDK where it belongs in unitree_ros_to_real
- git clone this repository
- build the packages (catkin_make / catkin build)

## Overview:
The ROS Package a1_controller consists of the necessary scripts to control the robot in Gazebo.
- A1_kinematics.py: (direct/inverse) kinematics library. We figured out the kinematic model using the Denavit-Hartenberg-Transformation.
![Alt Text](https://i.imgur.com/f0Jjd32.png)

The Above image is a visualization of the foot workspace and redundant positions (those with multiple viable joint configurations colored in blue) in RViz using Pointclouds.
If you want to play around with the foot workspace you can launch:
```
roslaunch a1_controller visualize_workspace.launch
```

## Usage:
Launch gazebo simulation and low-level controllers (for motorCMD message):

```
roslaunch unitree_gazebo normal.launch rname:=a1
```

If you were now to apply the instructions and provided nodes from Unitree...

![Alt Text](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExa3VyejJtYjhxOGlwejRmaWU3ZDdmY2tzNWRlNTJqYTNhODRneDZvdSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/WuXrsGJsW7vYuUkg8t/giphy.gif)

...there's either standing still or flying around the origin.

## Run an actual controller (ours):

```
rosrun a1_description wholebody_kinematics_demo.py
```
and
```
rosrun a1_description pose_pub_gui.py
```

This script uses the controllers initialized in the normal.launch by Unitree and our inverse kinematics model to showcase the translation/rotation of the robot's body in all 6 movement axes. 
We set up a small Pygame window to use a joystick GUI for easy and intuitive input.


![Alt Text](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExbnZvbW96OWoyZ3BsMGpmNnh5ZDRqdnE4bmU1aTRyYWFueHByajZ6YyZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/eOXNwK2yL2ZI6g3NbY/giphy.gif)

The Gazebo environment and the kinematics demo can be launched using:
```
roslaunch a1_controller wholebody_kinematics_demo.launch
```



## Next "Steps"
We are developing scripts to enable the locomotion (not flying) of the A1 in Gazebo. A small preview of the crawling gait can be demonstrated by running:

```
rosrun a1_description gazebo_controller.py
```
