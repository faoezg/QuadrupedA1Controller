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
max_eff = 55

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
            y = 10*np.sin(np.pi*(t/(tgp/2))) - (self.Rh)
            z = 3*t
        else:
            y = -(self.Rh)
        
        if t > tgp/2:
            z = (tgp/2)*3 + 3*((tgp/2)-t) 
            
        print(y,z)
        x = 8.38      
        return x,y,z
        

        
class EffortPublisher:
    def __init__(self):
        rospy.init_node('effort_publisher', anonymous=True)
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.apply_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self.joint_states_callback)
        
        self.rate = rospy.Rate(200)
        
        self.positions = np.array([0,0,0,0,0,0,
                          0,0,0,0,0,0])
        
        self.velocities = np.array([0,0,0,0,0,0,
                          0,0,0,0,0,0])
                            
        #self.ik = A1_kinematics.InverseKinematics()
        
        #pos_thetas = A1_kinematics.calc_joint_angles(pos_fr)
        #thetas = pos_thetas[0]

        self.goal_pos = np.array([-1.5708,0,0.785398,-1.5708,0,0.785398,
                                 -1.5708,0,0.785398,-1.5708,0,0.785398])
        
        self.goal_vel = np.array([0,0,0,0,0,0,
                         0,0,0,0,0,0])
        
        self.base_height = 20
        self.length_offset = 0
        self.base_width = 8.38
        self.base_width_r = 8.38        
        
    def publish_efforts(self):
        tp = Trajectory_Planner()
        
        t = 0
        while not rospy.is_shutdown():
            downscaler = 300
            if 0<=t<50: # test length shift
                self.length_offset -= 100/(4*downscaler)
            elif 50<=t<150:
                self.length_offset += 100/(4*downscaler)
            elif 150<=t<200:
                self.length_offset -= 100/(4*downscaler)
            
            if 200<=t<250: # test squat
                self.base_height -= 50/downscaler
            elif 250<=t<350:
                self.base_height += 50/downscaler  
            elif 350<=t<400:
                self.base_height -= 50/downscaler
                
            if 400<=t<450: # test width shift
                self.base_width -= 50/downscaler
                self.base_width_r += 50/downscaler

            elif 450<=t<550:
                self.base_width += 50/downscaler  
                self.base_width_r -= 50/downscaler  
            elif 550<=t<600:
                self.base_width -= 50/downscaler        
                self.base_width_r += 50/downscaler  
            
            angles_l = A1_kinematics.calc_joint_angles([self.base_width, self.base_height, self.length_offset])
            angles_r = A1_kinematics.calc_joint_angles([self.base_width_r, self.base_height, self.length_offset], False)

            th0_l = angles_l[0][0]
            th2_l = angles_l[0][1]
            th3_l = angles_l[0][2]
            
            th0_r = angles_r[0][0]
            th2_r = angles_r[0][1]
            th3_r = angles_r[0][2]
            
            
            self.goal_pos = [th3_l, -th0_l, th2_l + np.pi/2, th3_r, th0_r, th2_r + np.pi/2,
                             th3_l, -th0_l, th2_l + np.pi/2, th3_r, th0_r, th2_r + np.pi/2]
            
            efforts = self.calculate_joint_effort()            
            
            for i, eff in enumerate(efforts):
                if(np.abs(eff) > max_eff): 
                    eff = np.sign(eff) * max_eff

                self.apply_effort(joint_names[i], eff, rospy.Time(0), rospy.Duration(0.01))

            t+=1
            t %= 600
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