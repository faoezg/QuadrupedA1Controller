#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import ApplyJointEffort
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from unitree_legged_msgs.msg import MotorCmd
import numpy as np
from matplotlib import pyplot as plt

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
    def __init__(self) -> None:
        self.Rh = 0.30
    
    """def step_trajectory(self, H, Sx, tgp, Ts, Ty, t):
        a = t - Ts/2
        tau = a/Ty
        
        
        
        if (t <= Ts/2):
            z = -self.Rh
        elif Ts/2 < t <= Ty + Ts/2:
                if a <= Ty/2:
                    z = 2*H*(tau - (1/(4*np.pi))*np.sin(4*np.pi*tau) - 1) - self.Rh
                if a > Ty/2:
                    z = -2*H*(tau - (1/(4*np.pi))*np.sin(4*np.pi*tau) - 1) - self.Rh
        elif  (t > Ty + Ts/2):
            z = -self.Rh
        

        if t <= Ts/2:
            x = np.sign(Sx)*(-Sx/Ts)*t
        elif Ts/2 < t <= Ty + Ts/2:
            x = np.sign(Sx)*(Sx*(tau - (np.sin(2*np.pi*tau)/2*np.pi)) - Sx/2)
            
        elif t > Ty + Ts/2:
            x = np.sign(Sx)*(((0.5*Sx)/(Ty+0.5*Ts-tgp))+ 0.5*Sx*(1 - ((Ty+0.5*Ts)/(Ty+0.5*Ts-tgp))))
            
        print(x,z, "time: ", t)
        
        return x,z"""
    
    def step_trajectory(self, H, Sx, tgp, t):
        
        # schwungphase
        if t <= tgp/2:
            y = 0.1*np.sin(np.pi*(t/(tgp/2))) - (self.Rh-0.10)
            x = 0.03*t
        else:
            y = -(self.Rh-0.1)
        
        if t > tgp/2:
            x = (tgp/2)*0.03 + 0.03*((tgp/2)-t) 
            
        print(x,y)      
        return x,y
        

        
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
        self.ik = TwoLink_Kinematics()
        th1, th2 = self.ik.inverse_kinematics(X,Y)

        self.goal_pos = np.array([th2, 0, th1, th2, 0, th1, 
                                  th2, 0, th1, th2, 0, th1])
        
        self.goal_vel = np.array([0,0,0,0,0,0,
                         0,0,0,0,0,0])
        
        
    def publish_efforts(self):
        tp = Trajectory_Planner()
        
        """ CONTROL LOOP """
        t = 0
        while not rospy.is_shutdown():
            
            """ calculate next positions """
            if t % 10 == 0:
                
                for i in range(0,4):
                    for j in range(0,3):
                        # operator space goal
                        x,y = tp.step_trajectory(0.1,0.1,10,t/10)
                        # joint space goal
                        th1, th2 = self.ik.inverse_kinematics(x,y)
                        # add thetas to goal positions
                        self.goal_pos = [th2, 0, th1, th2, 0, th1, 
                                        th2, 0, th1, th2, 0, th1]
                        

            """ calculate and publish efforts """
            efforts = self.calculate_joint_effort()            
            
            for i, eff in enumerate(efforts):
                if(np.abs(eff) > max_eff): 
                    eff = np.sign(eff) * max_eff

                self.apply_effort(joint_names[i], eff, rospy.Time(0), rospy.Duration(0.01))

            t+=1
            t %=100
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
    """step_height = 0.1
    step_length = 0.1
    tgp = 100
    

    # Initialize the trajectory planner
    planner = Trajectory_Planner()

    # Create an array of time values
    t_values = np.linspace(0, tgp, 1000)

    #   Calculate x and z values for each time value
    x_values = []
    z_values = []
    for t in t_values:
        x, z = planner.step_trajectory(step_height, step_length, tgp, t)
        x_values.append(x)
        z_values.append(z)

    # Plot the trajectory
    plt.plot(x_values, z_values)
    plt.xlabel('x')
    plt.ylabel('z')
    plt.title('Trajectory')
    plt.grid(True)
    plt.show()"""
    
    try:
        effort_publisher = EffortPublisher()
        effort_publisher.publish_efforts()
    except rospy.ROSInterruptException:
        pass