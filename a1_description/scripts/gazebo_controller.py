#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import ApplyJointEffort
from sensor_msgs.msg import JointState
import numpy as np
from matplotlib import pyplot as plt
import A1_kinematics

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



class Trajectory_Planner:
    def __init__(self) -> None:
        self.Rh = 30
    
    def step_trajectory(self, H, Sx, tgp, t):
        
        # schwungphase
        if t <= tgp/2:
            y = 0.1*np.sin(np.pi*(t/(tgp/2))) - (self.Rh-0.10)
            z = 0.03*t
        else:
            y = -(self.Rh-0.1)
        
        if t > tgp/2:
            z = (tgp/2)*0.03 + 0.03*((tgp/2)-t) 
            
        print(y,z)      
        return y,z
        

        
class EffortPublisher:
    def __init__(self):
        rospy.init_node('effort_publisher', anonymous=True)
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.apply_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/hihi/joint_states", JointState, self.goal_state_callback)
        
        self.rate = rospy.Rate(100)
        
        self.positions = np.array([0,0,0,0,0,0,
                          0,0,0,0,0,0])
        
        self.velocities = np.array([0,0,0,0,0,0,
                          0,0,0,0,0,0])
        
                    

        #pos_fr = [13.857525967328343, 0.07049528719526688, 38.41996789425385]
        #self.X = pos_fr[2]
        #self.ik = A1_kinematics.InverseKinematics()
        
        #pos_thetas = A1_kinematics.calc_joint_angles(pos_fr)
        #thetas = pos_thetas[0]

        self.goal_pos = np.array([0,0,0,0,0,0,
                                  0,0,0,0,0,0])
        
        self.goal_vel = np.array([0,0,0,0,0,0,
                         0,0,0,0,0,0])
        
        
    def publish_efforts(self):
        tp = Trajectory_Planner()
        
        t = 0
        while not rospy.is_shutdown():
            
            #if t % 10 == 0:

                # operator space goal
                #Y,Z = tp.step_trajectory(0.1,0.1,10,t/10)
                
                # joint space goal
                #pos_thetas = A1_kinematics.calc_joint_angles([self.X,Y,Z])
                #thetas = pos_thetas[0]
                # add thetas to goal positions
                #self.goal_pos = [thetas[2],0,thetas[1],thetas[2],0,thetas[1],
                #                 thetas[2],0,thetas[1],thetas[2],0,thetas[1]]

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
        print(A1_kinematics.get_pw(self.positions[1 + 9],self.positions[0 + 9],self.positions[2 + 9]))
    
    def calculate_joint_effort(self):
        position_error = np.subtract(self.goal_pos,self.positions)
        velocity_error = np.subtract(self.goal_vel,self.velocities)

        control_effort = np.add(Kp * position_error,Kd * velocity_error)
        return control_effort
    
    def goal_state_callback(self, data):
        positions = data.position
        self.desired_pos_thigh = positions[4]
        self.desired_pos_calf = positions[5]
        
        self.goal_pos = [0,0,0,
                         0,0,0,
                         0,0,0,
                         self.desired_pos_calf,0,self.desired_pos_thigh]


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