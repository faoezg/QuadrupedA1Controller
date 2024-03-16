#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import ApplyJointEffort
from sensor_msgs.msg import JointState
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

def get_position(q1, q2):
    """Solves the direct Kinematics for a 2-Link-Planar Leg"""
    Px = -l1*np.sin(q1) - l2*np.sin(q1+q2)
    Py = -l1*np.cos(q1) - l2*np.cos(q1+q2)
    
    return Px,Py

def equations_of_motion(q1,q2,dq1,dq2,ddq1,ddq2):

    tau1 = (ddq1*(m1*(lm1**2) + m2*(l1**2) + m2*(lm2**2) + 2*m2*l1*lm2*np.cos(q2) + I1 + I2) 
            + ddq2*(m2*(lm2**2) + m2*l1*lm2*np.cos(q2) + I2)
            - 2*dq1*dq2*m2*l1*lm2*np.sin(q2) - (dq2**2)*m2*l1*lm2*np.sin(q2)
            + g*m1*lm1*np.sin(q1) + g*m2*l1*np.sin(q1) + g*m2*lm2*np.sin(q1+q2))
    
    tau2 = (ddq1*(m2*(lm2**2) + m2*l1*lm2*np.cos(q2)+ I2) + ddq2*(m2*(lm2**2)+ I2)
            + (dq1**2)*m2*l1*lm2*np.sin(q2) + g*m2*lm2*np.sin(q1+q2))
    #print(tau1, tau2)
    return tau1, tau2


class EffortPublisher:
    def __init__(self):
        rospy.init_node('effort_publisher', anonymous=True)
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.apply_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/hihi/joint_states", JointState, self.goal_state_callback)

        self.rate = rospy.Rate(25)
        
        self.positions = []
        self.velocities = []
        self.efforts = []
        
        self.desired_pos_thigh = 0             #actual robots joint values
        self.desired_pos_calf = -1.8
        self.desired_vel_thigh = 0.0
        self.desired_vel_calf = 0.0
        
        self.current_pos_thigh = 0
        self.current_pos_calf = 0
        self.current_vel_thigh = 0
        self.current_vel_calf = 0
        self.current_eff_thigh = 0
        self.current_eff_calf = 0

    def publish_efforts(self):
        while not rospy.is_shutdown():
            # get "current params" (position/velocity/effort) from /a1_gazebo/joint_states
            
            # calculate next effort using "current params"
            position_error_calf = self.desired_pos_calf - self.current_pos_calf
            position_error_thigh = self.desired_pos_thigh - self.current_pos_thigh
            velocity_error_calf = self.desired_vel_calf - self.current_vel_calf
            velocity_error_thigh = self.desired_vel_thigh - self.current_vel_thigh
            
            control_effort_calf = Kp * position_error_calf + Kd* velocity_error_calf
            control_effort_thigh = Kp * position_error_thigh + Kd* velocity_error_thigh
         
            calc_tau1, calc_tau2 = equations_of_motion(position_error_thigh, position_error_calf, self.current_vel_thigh,
                                                       self.current_vel_calf, self.current_eff_thigh, self.current_eff_calf)
            calc_tau1 *= 50
            calc_tau2 *= 125
            
            
            if(calc_tau1 > 33.5 or calc_tau1 < -33.5):
                calc_tau1 = np.sign(calc_tau1) * 33.5
                
            if(calc_tau2 > 33.5 or calc_tau2 < -33.5):
                calc_tau2 = np.sign(calc_tau2) * 33.5
                
            if(control_effort_calf > 33.5 or control_effort_calf < -33.5):
                control_effort_calf = np.sign(control_effort_calf) * 33.5
                
            if(control_effort_thigh > 33.5 or control_effort_thigh < -33.5):
                control_effort_thigh = np.sign(control_effort_thigh) * 33.5

            self.apply_effort('FL_thigh_joint', control_effort_thigh, rospy.Time(0), rospy.Duration(0.04)) # tau1
            self.apply_effort('FL_calf_joint', control_effort_calf, rospy.Time(0), rospy.Duration(0.04)) # tau2
            #print("CTRL Effort", control_effort_thigh, control_effort_calf)
            self.rate.sleep()
        
    def joint_states_callback(self, data):
        positions = data.position
        velocities = data.velocity
        efforts = data.effort
        self.current_pos_calf = positions[0]
        self.current_pos_thigh = positions[2]
        self.current_vel_calf = velocities[0]
        self.current_vel_thigh = velocities[2]
        self.current_eff_calf = efforts[0]
        self.current_eff_thigh = efforts[2]
        
        
         
        x,y = get_position(positions[2], positions[0])
        #print(f"----------------Front Left------------\nPos: thigh={positions[2]}, calf={positions[0]} \nVel: thigh={velocities[2]}, calf={velocities[0]} \nEff: thigh={efforts[2]}, calf={efforts[0]}")
        print(f"POSITION (X,Y)= {x,y}")
        
    def goal_state_callback(self, data):
        positions = data.position
        self.desired_pos_thigh = positions[4]
        self.desired_pos_calf = positions[5]
        #print(positions)
        

if __name__ == '__main__':
    try:
        effort_publisher = EffortPublisher()
        effort_publisher.publish_efforts()
    except rospy.ROSInterruptException:
        pass
