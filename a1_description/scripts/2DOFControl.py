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
max_eff = 33.5

joint_names = ["FL_calf_joint","FL_hip_joint","FL_thigh_joint",
                "FR_calf_joint","FR_hip_joint","FR_thigh_joint",
                "RL_calf_joint","RL_hip_joint","RL_thigh_joint",
                "RR_calf_joint","RR_hip_joint","RR_thigh_joint"]
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
        
        sin_th1 = num_s/denom
        cos_th1 = num_c/denom
                
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
        self.kinematics = TwoLink_Kinematics()
        
    def fixed_point(self, position):
        """Stand still at a certain point in x-y plane"""
        
        #Calculate IK for that Position
        th1, th2 = self.kinematics.inverse_kinematics(*position)
        
        if self.legID == 0:  #front left
            self.EPub.goal_pos[0] = th2  # calf
            self.EPub.goal_pos[2] = th1  # thigh
        if self.legID == 1:  #front right
            self.EPub.goal_pos[3] = th2
            self.EPub.goal_pos[5] = th1
        if self.legID == 2:  #rear left
            self.EPub.goal_pos[6] = th2
            self.EPub.goal_pos[8] = th1
        if self.legID == 3:  #rear right
            self.EPub.goal_pos[9] = th2
            self.EPub.goal_pos[11] = th1

        
    def swing_phase(self,x,y,time):
        time = time % self.phaseLen
        
        if(time < self.phaseLen/2):
            x+=0.004*time
            y += 0.05* np.sin(time/(self.phaseLen/4))

        else:
            x += 0.004*self.phaseLen/2
            x-=0.004*(time - self.phaseLen/2)
        
                
        
        th1, th2 = self.kinematics.inverse_kinematics(x,y)
            
        
        if self.legID == 0:  #front left
                self.EPub.goal_pos[0] = th2  # calf
                self.EPub.goal_pos[2] = th1  # thigh
        if self.legID == 1:  #front right
                self.EPub.goal_pos[3] = th2
                self.EPub.goal_pos[5] = th1
        if self.legID == 2:  #rear left
                self.EPub.goal_pos[6] = th2
                self.EPub.goal_pos[8] = th1
        if self.legID == 3:  #rear right
                self.EPub.goal_pos[9] = th2
                self.EPub.goal_pos[11] = th1
        return    
    
    def swing_func(self,x,y,t):

        t = t % self.phaseLen
        if self.legID == 0:
            if 0<= t <25:
                y = 0.05*np.abs(np.sin(t*np.pi/25)) + y
                x+=0.004*t
                
        if self.legID == 1:
            if 25<= t <50:
                y = 0.05*np.abs(np.sin(t*np.pi/25)) + y
                x+=0.004*(t-25)
            
        if self.legID == 3:    
            if 50<= t <75:
                y = 0.05*np.abs(np.sin(t*np.pi/25)) + y
                x+=0.004*(t-50)

        if self.legID == 2:
            if 75<= t <100:
                y = 0.05*np.abs(np.sin(t*np.pi/25)) + y
                x+=0.004*(t-75)
            
            
            
        th1, th2 = self.kinematics.inverse_kinematics(x,y)
        if self.legID == 0:  #front left
                self.EPub.goal_pos[0] = th2  # calf
                self.EPub.goal_pos[2] = th1  # thigh
        if self.legID == 1:  #front right
                self.EPub.goal_pos[3] = th2
                self.EPub.goal_pos[5] = th1
        if self.legID == 2:  #rear left
                self.EPub.goal_pos[6] = th2
                self.EPub.goal_pos[8] = th1
        if self.legID == 3:  #rear right
                self.EPub.goal_pos[9] = th2
                self.EPub.goal_pos[11] = th1
        
        
class EffortPublisher:
    def __init__(self):
        rospy.init_node('effort_publisher', anonymous=True)
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.apply_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self.joint_states_callback)
        self.rate = rospy.Rate(100)
        
        self.positions = np.array([0,0,0,0,0,0,
                          0,0,0,0,0,0])
        
        self.velocities = np.array([0,0,0,0,0,0,
                          0,0,0,0,0,0])
        
        
        X = -0.175
        Y = -0.3
        ik = TwoLink_Kinematics()
        th1, th2 = ik.inverse_kinematics(X,Y)

        self.goal_pos = np.array([th2, 0, th1, th2, 0, th1, 
                                  th2, 0, th1, th2, 0, th1])
        
        self.goal_vel = np.array([0,0,0,0,0,0,
                         0,0,0,0,0,0])
        
        self.FL_planner = Trajectory_Planner(0,100,self)
        self.FR_planner = Trajectory_Planner(1,100,self)
        self.RL_planner = Trajectory_Planner(2,100,self)
        self.RR_planner = Trajectory_Planner(3,100,self)
        
        
    def publish_efforts(self):
        X = -0.175
        Y = -0.25
        """ CONTROL LOOP """
        t = 0
        while not rospy.is_shutdown():
            
            """ calculate and publish efforts """
            efforts = self.calculate_joint_effort()            
            
            for i, eff in enumerate(efforts):
                if(np.abs(eff) > max_eff): 
                    eff = np.sign(eff) * max_eff

                self.apply_effort(joint_names[i], eff, rospy.Time(0), rospy.Duration(0.01))

            """ calculate next positions """
            
            #self.FL_planner.fixed_point([X, Y])
            #self.FR_planner.fixed_point([-0.18, Z])
            """self.FR_planner.swing_func(X,Y,t)
            self.FL_planner.swing_func(X,Y,t)

            self.RR_planner.swing_func(X,Y,t)
            self.RL_planner.swing_func(X,Y,t)"""
            self.FR_planner.swing_phase(X,Y,t)
            self.FL_planner.swing_phase(X,Y,t+50)

            self.RR_planner.swing_phase(X,Y,t+50)
            self.RL_planner.swing_phase(X,Y,t)

            t+=1
            self.rate.sleep()
         
        
    def joint_states_callback(self, data):
        self.positions = data.position
        self.velocities = data.velocity
    
    def calculate_joint_effort(self):
        position_error = np.subtract(self.goal_pos,self.positions)
        velocity_error = np.subtract(self.goal_vel,self.velocities)

        control_effort = np.add(Kp * position_error,Kd * velocity_error)
        return control_effort


if __name__ == '__main__':
    try:
        effort_publisher = EffortPublisher()
        effort_publisher.publish_efforts()
    except rospy.ROSInterruptException:
        pass
