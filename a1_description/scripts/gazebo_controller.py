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

leg_offset_y = 0.1805*100
leg_offset_x = 0.047*100

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
    
def calc_correct_thetas(position, prev_ths, isLeft):
    possible_joint_angles = A1_kinematics.calc_joint_angles(position, isLeft)

    if len(possible_joint_angles) == 0:
        print("no possible angles could be found for this Positon. Stop")
        return prev_ths                  

    min_val = calc_joint_difference(prev_ths,possible_joint_angles[0])
    min_at = 0
    
    for i in range(1, len(possible_joint_angles)):
        difference = calc_joint_difference(prev_ths,possible_joint_angles[i])
        if(difference < min_val):
            min_val = difference
            min_at = i

    return possible_joint_angles[min_at]
    

def calc_joint_difference(prev_ths, cur_ths):  # computes a cumulative absolute difference between joint configs
    diff_th0 = np.abs(prev_ths[0] - cur_ths[0])

    diff_th2 = np.abs(prev_ths[1] - cur_ths[1])

    diff_th3 = np.abs(prev_ths[2] - cur_ths[2])

    return diff_th0 + diff_th2 + diff_th3

    
def global_foot_pos(id, position):  # calculates the foot position in reference to the base link of the quadruped

    if id == 0:  # FL
        position[0] -= leg_offset_x
        position[2] -= leg_offset_y
    
    elif id == 1:  # FR
        position[0] += leg_offset_x
        position[2] -= leg_offset_y
    
    elif id == 2:  # RL
        position[0] -= leg_offset_x
        position[2] += leg_offset_y
    
    elif id == 3:  # RR
        position[0] += leg_offset_x
        position[2] += leg_offset_y

    return position

def local_foot_pos(id, position):  # does the inverse to the above function
    if id == 0:  # FL
        position[0] += leg_offset_x
        position[2] += leg_offset_y
    
    elif id == 1:  # FR
        position[0] -= leg_offset_x
        position[2] += leg_offset_y
    
    elif id == 2:  # RL
        position[0] += leg_offset_x
        position[2] -= leg_offset_y
    
    elif id == 3:  # RR
        position[0] -= leg_offset_x
        position[2] -= leg_offset_y

    return position
    
def apply_rpy(x,y,z,roll,pitch,yaw):
    """Function to rotate global positions according to desired base-rpy"""
    
    rotate_y = np.matrix([[np.cos(yaw), 0, np.sin(yaw)],
                          [0, 1, 0],
                          [-np.sin(yaw), 0, np.cos(yaw)]])
 
    rotate_x = np.matrix([[1, 0, 0],
                          [0, np.cos(pitch), -np.sin(pitch)],
                          [0, np.sin(pitch), np.cos(pitch)]])    
    
    rotate_z = np.matrix([[np.cos(roll), -np.sin(roll), 0],
                          [np.sin(roll), np.cos(roll), 0],
                          [0, 0, 1]])
    
    vector = rotate_z @ rotate_x @ rotate_y @ np.matrix([[x],[y],[z]])
    
    x_new = vector.item(0)
    y_new = vector.item(1)
    z_new = vector.item(2)
    return [x_new, y_new, z_new]
    
    
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
                            

        self.goal_pos = np.array([-1.5708, 0, 0.785398, -1.5708, 0, 0.785398,
                                 -1.5708, 0, 0.785398, -1.5708, 0, 0.785398])
        
        self.goal_vel = np.array([0,0,0,0,0,0,
                         0,0,0,0,0,0])
        
        self.hip_to_toe_pos = [[-8.38, 22.5, 0],  # FL
                               [8.38, 22.5, 0],  # FR
                               [-8.38, 22.5, 0],  # RL
                               [8.38, 22.5, 0]]  # RR

        self.yaw = 0  # yaw rotation of body
        self.pitch = 0  # pitch rotation 
        self.roll = 0 # roll rotation
        
        self.global_positions = [[0,0,0],
                                 [0,0,0],
                                 [0,0,0],
                                 [0,0,0]]
        
        
    def publish_efforts(self):
        t = 0
        while not rospy.is_shutdown():
            downscaler = 300
            
            xyz_mode = False
            
            if 0<=t<50: # test length shift / pitch rotation        
                if xyz_mode:        
                    for i in range(0,4):
                        self.hip_to_toe_pos[i][2] += 50/downscaler 
                else:
                    self.pitch += 0.00174533/10             
            elif 50<=t<150:
                if xyz_mode:        
                    for i in range(0,4):
                        self.hip_to_toe_pos[i][2] -= 50/downscaler 
                else:
                    self.pitch += 0.00174533/10       
            elif 150<=t<200:
                if xyz_mode:        
                    for i in range(0,4):
                        self.hip_to_toe_pos[i][2] += 50/downscaler 
                else:
                    self.pitch += 0.00174533/10
                    
                             
            if 200<=t<250: # test squat / height shift / yaw movement
                if xyz_mode:
                    for i in range(0,4):
                        self.hip_to_toe_pos[i][1] += 50/downscaler 
                else:
                    self.yaw += 0.00174533/10             
            elif 250<=t<350:
                if xyz_mode:
                    for i in range(0,4):
                        self.hip_to_toe_pos[i][1] -= 50/downscaler 
                else:
                    self.yaw += 0.00174533/10   
            elif 350<=t<400:
                if xyz_mode:
                    for i in range(0,4):
                        self.hip_to_toe_pos[i][1] += 50/downscaler 
                else:
                    self.yaw += 0.00174533/10
                
                
            if 400<=t<450: # test width shift / roll movement   
                if xyz_mode:            
                    for i in range(0,4):
                        self.hip_to_toe_pos[i][0] += 50/downscaler        
                else:
                    self.roll += 0.00174533/8.8
                       
            elif 450<=t<550:
                if xyz_mode:            
                    for i in range(0,4):
                        self.hip_to_toe_pos[i][0] -= 50/downscaler        
                else:
                    self.roll += 0.00174533/8.8  
            
            elif 550<=t<600:
                if xyz_mode:            
                    for i in range(0,4):
                        self.hip_to_toe_pos[i][0] += 50/downscaler        
                else:
                    self.roll += 0.00174533/8.8
                
   
            for legIdx in range(0,4):
                # calculate global positions (base to foot)
                self.global_positions[legIdx] = global_foot_pos(legIdx, self.hip_to_toe_pos[legIdx])
                
                # apply RPY via rotation matrix
                self.global_positions[legIdx] = apply_rpy(self.global_positions[legIdx][0], self.global_positions[legIdx][1], 
                                                          self.global_positions[legIdx][2], self.roll, self.pitch, self.yaw)
                
                # set new local position (hip to foot)
                self.hip_to_toe_pos[legIdx] = local_foot_pos(legIdx,self.global_positions[legIdx])
                
                # get current leg angles from robot
                current_ths = [self.positions[legIdx*3 + 1], self.positions[legIdx*3 + 2], self.positions[legIdx*3]]
                
                # calculate closest solution for next position
                goal_ths  = calc_correct_thetas([self.hip_to_toe_pos[legIdx][0], self.hip_to_toe_pos[legIdx][1], self.hip_to_toe_pos[legIdx][2]],
                                                current_ths, legIdx % 2 == 0)
                
                # set goal angles for corresponding leg
                self.goal_pos[legIdx*3] = goal_ths[2]
                self.goal_pos[legIdx*3 + 1] = goal_ths[0]
                self.goal_pos[legIdx*3 + 2] = goal_ths[1] + np.pi/2     
            
            efforts = self.calculate_joint_effort()            
            for i, eff in enumerate(efforts):
                if(np.abs(eff) > max_eff): 
                    eff = np.sign(eff) * max_eff

                self.apply_effort(joint_names[i], eff, rospy.Time(0), rospy.Duration(0.01))

            t+=1
            if t==599:
                xyz_mode = not xyz_mode
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

    try:
        effort_publisher = EffortPublisher()
        effort_publisher.publish_efforts()
    except rospy.ROSInterruptException:
        pass