
# QuadrupedA1Controller

A collection of python scripts to handle the kinematics of the Unitree A1 Quadruped robot
## Installation


Important Packages:
- unitree_ros: 
    https://github.com/unitreerobotics/unitree_ros.git

- unitree_ros_to_real:
    https://github.com/unitreerobotics/unitree_ros_to_real.git

- unitree_legged_sdk v3.3.4:
    https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/3.3.4

Installation:
- in ~/catkin_ws/src/
- git clone the unitree_ros repository
- inside unitree_ros: copy contents of unitree_ros_to_real into the corresponding folder
- download unitree_legged_sdk v3.3.4, unzip and place the sdk into where it belongs in unitree_ros_to_real
- build the catkin packages 
- either remove or replace the a1_description package with our repository

## Usage:
launch gazebo simulation and low level controllers (for motorCMD message):

```
roslaunch unitree_gazebo normal.launch rname:=a1
```

if you were now to apply the instructions and provided nodes from unitree...

![Alt Text](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExa3VyejJtYjhxOGlwejRmaWU3ZDdmY2tzNWRlNTJqYTNhODRneDZvdSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/WuXrsGJsW7vYuUkg8t/giphy.gif)

...there's either standing still or flying around the origin.

## Run the actual controller (ours):

```
rosrun a1_description wholebody_kinematics_demo.py
```

This script uses the controllers initialised in the normal.launch by unitree and our inverse kinematics model to showcase the translation/rotation of the robots body in all 6 movements axes. 
For easy and intuitive input we provided a small pygame window to use a joystick GUI.