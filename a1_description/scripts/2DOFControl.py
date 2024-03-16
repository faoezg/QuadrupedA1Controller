#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import ApplyJointEffort
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from unitree_legged_msgs.msg import MotorCmd
import numpy as np

l1 = l2 = 0.20
lm1 = 0.030323
lm2 = 0.096933
g = 9.81
m1 = 0.888 # thigh mass
m2 = 0.151 # calf mass
I1 = 0.001110200 # thigh Izz
I2 = 0.000031158 # calf Izz
Kp = 300
Kd = 8

th1_max = 4.18
th1_min = -1.05
th2_max = -0.92
th2_min = -2.69

"""Snippet from JointStates Message:
name: 
  - FL_calf_joint
  - FL_hip_joint
  - FL_thigh_joint
  - FR_calf_joint
  - FR_hip_joint
  - FR_thigh_joint
  - RL_calf_joint
  - RL_hip_joint
  - RL_thigh_joint
  - RR_calf_joint
  - RR_hip_joint
  - RR_thigh_joint
position: [-1.8055664164916871, 0, 0, -1.8055664164916871, 0, 0, 
           -1.8055664164916871, 0, 0, -1.8055664164916871, 0, 0]
"""

class TwoLink_Kinematics:
    def __init__(self):
        self.a1 = l1
        self.a2 = l2
        
    def direct_kinematics(self, th2, th1):
        """Solves the direct Kinematics for a 2-Link-Planar Leg"""
        Px = -self.a1*np.sin(th1) - self.a2*np.sin(th1+th2)
        Py = -self.a1*np.cos(th1) - self.a2*np.cos(th1+th2)
        return Px,Py
        
    def calculate_th2(self, posX, posY):
        cos_th2 = (posX**2 + posY**2 - self.a1**2 - self.a2**2) / (2*self.a1*self.a2)
        sin_th2_0 = np.sqrt(1-cos_th2**2)
        sin_th2_1 = -np.sqrt(1-cos_th2**2) 
        th2_0 = np.arctan2(sin_th2_0, cos_th2) 
        th2_1 = np.arctan2(sin_th2_1, cos_th2) 
        return th2_1

    
    def calculate_th1(self, posX, posY, th2):
        num_s = posX*(-self.a1 - self.a2*np.cos(th2)) + self.a2*np.sin(th2)*posY
        num_c = posY*(-self.a1 - self.a2*np.cos(th2)) - self.a2*np.sin(th2)*posX
        denom = self.a1**2 + self.a2**2 + 2*self.a1*self.a2*np.cos(th2)
        
        sin_th1 = num_s
        cos_th1 = num_c
                
        th1 = np.arctan2(sin_th1, cos_th1)
        return th1
    
    def inverse_kinematics(self,posX, posY):
        th2 = self.calculate_th2(posX, posY)
        th1 = self.calculate_th1(posX, posY, th2)
        
        if th1_min < th1 < th1_max and th2_min < th2 < th2_max:
            return th1, th2
        return None

class Trajectory_Planner:
    def __init__(self, legID, phaseLen, EPub) -> None:
        self.legID = legID
        self.phaseLen = phaseLen
        self.EPub = EPub
    
    def stand_phase(self, position):
        """Stand still at a certain point in x-y plane"""
        self.EPub.goal_pos = np.add(self.EPub.goal_pos, [0,0,0,
                                                         0,0,0,
                                                         0,0,0,
                                                         0,0,0])
        
        
        
class EffortPublisher:
    def __init__(self):
        rospy.init_node('effort_publisher', anonymous=True)
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.apply_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self.joint_states_callback)

        """   DIDNT WORK AHAHA
        command_topics = [
                          "/a1_gazebo/FL_calf_controller/command",  
                          "/a1_gazebo/FL_hip_controller/command",
                          "/a1_gazebo/FL_thigh_controller/command",
                          "/a1_gazebo/FR_calf_controller/command",
                          "/a1_gazebo/FR_hip_controller/command",
                          "/a1_gazebo/FR_thigh_controller/command",
                          "/a1_gazebo/RL_calf_controller/command",
                          "/a1_gazebo/RL_hip_controller/command",
                          "/a1_gazebo/RL_thigh_controller/command",
                          "/a1_gazebo/RR_calf_controller/command",
                          "/a1_gazebo/RR_hip_controller/command",
                          "/a1_gazebo/RR_thigh_controller/command",
                          ]

        self.publishers = []
        for i in range(len(command_topics)):
            self.publishers.append(rospy.Publisher(command_topics[i], MotorCmd, queue_size = 0))
        
        """
        self.rate = rospy.Rate(20)
        
        self.positions = np.array([0,0,0,0,0,0,
                          0,0,0,0,0,0])
        
        self.velocities = np.array([0,0,0,0,0,0,
                          0,0,0,0,0,0])
        
        self.goal_pos = np.array([-1.0, 0, 0.5, -1, 0, 0.5, 
                                  -1.0, 0, 0.5, -1, 0, 0.5])
        
        self.goal_vel = np.array([0,0,0,0,0,0,
                         0,0,0,0,0,0])
        
        
    def publish_efforts(self):
        joint_names = ["FL_calf_joint","FL_hip_joint","FL_thigh_joint",
                       "FR_calf_joint","FR_hip_joint","FR_thigh_joint",
                       "RL_calf_joint","RL_hip_joint","RL_thigh_joint",
                       "RR_calf_joint","RR_hip_joint","RR_thigh_joint"]
        
        while not rospy.is_shutdown():
            # get "current params" (position/velocity/effort) from /a1_gazebo/joint_states
            
            efforts = self.calculate_joint_effort()
            
            for i, eff in enumerate(efforts):
                
                if(np.abs(eff) > 33.5):  # check wether calculated effort is bigger than the robots max
                    eff = np.sign(eff) * 33.5

                self.apply_effort(joint_names[i], eff, rospy.Time(0), rospy.Duration(0.05))

            self.rate.sleep()
        
    def joint_states_callback(self, data):
        self.positions = data.position
        self.velocities = data.velocity
    
    def calculate_joint_effort(self):
        position_error = np.subtract(self.goal_pos,self.positions)
        velocity_error = np.subtract(self.goal_vel,self.velocities)
        #print("Position Error:", position_error)
        control_effort = np.add(Kp * position_error,Kd * velocity_error)
        return control_effort

if __name__ == '__main__':
    try:
        effort_publisher = EffortPublisher()
        effort_publisher.publish_efforts()
    except rospy.ROSInterruptException:
        pass
