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

th1_max = 4.18
th1_min = -1.05
th2_max = -0.92
th2_min = -2.69


def equation_of_motion(q1,q2,dq1,dq2,ddq1,ddq2):
    B = np.array([[[m1*(lm1**2) + m2*(l1**2 + lm2**2 + 2*l1*lm2*np.cos(q2))+ I1 + I2],[m2*(lm2**2 + l1*lm2*np.cos(q2) + I2)]],
                 [[m2*(lm2**2 + l1*lm2*np.cos(q2) + I2)],[m2*(lm2**2) + I2]]])

    C = np.array([[[-dq2*m2*l1*lm2*np.sin(q2)],[-(dq1 + dq2)*m2*l1*lm2*np.sin(q2)]],
                 [[dq1*m1*l1*lm2*np.sin(q2)],[0]]])

    G = np.array([[g*m1*lm1*np.sin(q1) + g*m2*l1*np.sin(q1) + g*m2*lm2*np.sin(q1 + q2)],
                 [g*m2*lm2*np.sin(q1+q2)]])   #kann sein, dass die matrix falsch ist
    ddqs = np.array([[ddq1],[ddq2]])
    dqs = np.array([[dq1],[dq2]])
    
    taus = B * ddqs + C * dqs + G

    return taus[0][0],taus[0][1]

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
        print(th2_0, th2_1)
        return th2_1

    
    def calculate_th1(self, posX, posY, th2):
        num_s = posX*(-self.a1 - self.a2*np.cos(th2)) + self.a2*np.sin(th2)*posY
        num_c = posY*(-self.a1 - self.a2*np.cos(th2)) - self.a2*np.sin(th2)*posX
        denom = self.a1**2 + self.a2**2 + 2*self.a1*self.a2*np.cos(th2)
        
        sin_th1 = num_s
        cos_th1 = num_c
                
        th1 = np.arctan2(sin_th1, cos_th1)
        return th1
        
class EffortPublisher:
    def __init__(self):
        rospy.init_node('effort_publisher', anonymous=True)
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.apply_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/hihi/joint_states", JointState, self.goal_state_callback)

        self.rate = rospy.Rate(20)
        
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
        
        # init kinematics:
        self.kinematics = TwoLink_Kinematics()
        self.X, self.Y = [-0.18, -0.18] #specify goal position in x,y plane
        
        # calculate necessary joint config to achieve that position:
        
        self.desired_pos_calf = self.kinematics.calculate_th2(self.X,self.Y) # RETURNS ONLY THE NEGATIVE RESULT
        self.desired_pos_thigh = self.kinematics.calculate_th1(self.X,self.Y,self.desired_pos_calf) 

        
    def publish_efforts(self):
        while not rospy.is_shutdown():
            # get "current params" (position/velocity/effort) from /a1_gazebo/joint_states
            
            # calculate next effort using "current params"
            position_error_calf = self.desired_pos_calf - self.current_pos_calf
            position_error_thigh = self.desired_pos_thigh - self.current_pos_thigh
            velocity_error_calf = self.desired_vel_calf - self.current_vel_calf
            velocity_error_thigh = self.desired_vel_thigh - self.current_vel_thigh
            
            # aq = desired effort + (velocity error)*Kd + (position error)*Kp
            control_effort_calf = Kp * position_error_calf + Kd* velocity_error_calf
            control_effort_thigh = Kp * position_error_thigh + Kd* velocity_error_thigh
            
            
         
            """calc_tau1, calc_tau2 = equation_of_motion(position_error_thigh, position_error_calf, self.current_vel_thigh,
                                                       self.current_vel_calf, control_effort_thigh, control_effort_calf)
            calc_tau1 *= 10
            calc_tau2 *= 75
            
            
            if(calc_tau1 > 33.5 or calc_tau1 < -33.5):
                calc_tau1 = np.sign(calc_tau1) * 33.5
                
            if(calc_tau2 > 33.5 or calc_tau2 < -33.5):
                calc_tau2 = np.sign(calc_tau2) * 33.5"""
                
            if(control_effort_calf > 33.5 or control_effort_calf < -33.5):
                control_effort_calf = np.sign(control_effort_calf) * 33.5
                
            if(control_effort_thigh > 33.5 or control_effort_thigh < -33.5):
                control_effort_thigh = np.sign(control_effort_thigh) * 33.5

            self.apply_effort('FL_thigh_joint', control_effort_thigh, rospy.Time(0), rospy.Duration(0.05)) # tau1
            self.apply_effort('FL_calf_joint', control_effort_calf, rospy.Time(0), rospy.Duration(0.05)) # tau2
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

        
    def goal_state_callback(self, data):
        positions = data.position
        self.desired_pos_thigh = positions[4]
        self.desired_pos_calf = positions[5]
        

if __name__ == '__main__':
    try:
        effort_publisher = EffortPublisher()
        effort_publisher.publish_efforts()
    except rospy.ROSInterruptException:
        pass
