#!/usr/bin/env python3

import rospy
from tf import transformations
from sensor_msgs.msg import JointState, Imu
from unitree_legged_msgs.msg import MotorCmd
import numpy as np
import A1_kinematics
from Trajectory_Planner import Trajectory_Planner

# constants:
Kp = 300
Kd = 5

joint_names = ["FL_calf_joint","FL_hip_joint","FL_thigh_joint",
                "FR_calf_joint","FR_hip_joint","FR_thigh_joint",
                "RL_calf_joint","RL_hip_joint","RL_thigh_joint",
                "RR_calf_joint","RR_hip_joint","RR_thigh_joint"]

command_topics = ["/a1_gazebo/FL_calf_controller/command",
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
                  "/a1_gazebo/RR_thigh_controller/command"]
            
                  
    
class A1Controller:
    def __init__(self):
        rospy.init_node('joint_command_publisher', anonymous=True)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/trunk_imu", Imu, self.imu_callback)
        
        self.rate = rospy.Rate(25)
        self.positions = np.array([0,0,0,0,0,0,
                                   0,0,0,0,0,0])
        self.velocities = np.array([0,0,0,0,0,0,
                                    0,0,0,0,0,0])
                        
        #self.goal_pos = np.array([-1.5708, 0, 0.785398, -1.5708, 0, 0.785398,-1.5708, 0, 0.785398, -1.5708, 0, 0.785398])  # standing position
                                 
        self.goal_pos = np.array([-1.939770004895676, -0.26771448567122835, 1.2482740596122017, 
                                  -1.7291994402849626, -0.240265426734668, 0.8715556739001613, 
                                  -2.022709353102668, -0.27597424214704613, 1.147731458008173, 
                                  -1.8309775156913535, -0.2575122107675636, 0.9668397173376384])  # start of walking loop pose
        
        self.startup_pos = np.array([-2.6965310400372937, 0.49888734456542494, 1.120544218976467, 
                                    -2.6965319796256715, -0.4970180265271118, 1.1206134112047828, 
                                    -2.696527603682461, 0.4957650374287921, 1.1204999226739023, 
                                    -2.69653004841636, -0.49384031828850805, 1.1206527911125832])  # lying down pose
        
        self.hip_to_toe_pos = [[-0.0838, 0.225, 0.0],  # FL
                               [0.0838, 0.225, 0.0],  # FR
                               [-0.0838, 0.225, 0.0],  # RL
                               [0.0838, 0.225, 0.0]]  # RR

        self.yaw = 0.0  # yaw rotation of body
        self.pitch = 0  # pitch rotation 
        self.roll = 0 # roll rotation
        self.x_shift = 0.06  # amount of leaning to compensate COM for moving a leg 

        self.global_positions = [[0,0,0],
                                 [0,0,0],
                                 [0,0,0],
                                 [0,0,0]]
        self.publishers = []
        for topic in command_topics:
            self.publishers.append(rospy.Publisher(topic,MotorCmd, queue_size=0))
                
                
    def publish_commands(self):
        t = 0
        tp = Trajectory_Planner()
        step_period = 100
        
        # create and configure MotorCmd message 
        motor_command = MotorCmd()
        motor_command.mode = 10  
        motor_command.Kp = Kp
        motor_command.Kd = Kd 
        
        ## Startup sequence:
        print("Standing up")
        num_steps = 100
        step = (self.goal_pos - self.startup_pos)/num_steps
        
        for i in range(num_steps):
            for i in range(0, len(joint_names)):
                self.startup_pos[i] += step[i]
                motor_command.q = self.startup_pos[i]
    
                self.publishers[i].publish(motor_command)  
            self.rate.sleep()
        
        ## Movement Loop
        print("Start of Movement Loop")
        while not rospy.is_shutdown():
            
            for legIdx in range(0,4):
                # calculate next step:
                self.hip_to_toe_pos[legIdx] = tp.big_steppa(legIdx, self.hip_to_toe_pos[legIdx], 0.05, 0.1, step_period, t)
                
                # calculate global positions (base to foot)
                self.global_positions[legIdx] = tp.global_foot_pos(legIdx, self.hip_to_toe_pos[legIdx])
                
                # apply RPY via rotation matrix
                self.global_positions[legIdx] = tp.apply_rpy(self.global_positions[legIdx][0], 
                                                          self.global_positions[legIdx][1], 
                                                          self.global_positions[legIdx][2], 
                                                          self.roll, self.pitch, self.yaw)
                
                # set new local position (hip to foot)
                self.hip_to_toe_pos[legIdx] = tp.local_foot_pos(legIdx,self.global_positions[legIdx])
                
                # get current leg angles from robot
                current_ths = [self.positions[legIdx*3 + 1], 
                               self.positions[legIdx*3 + 2], 
                               self.positions[legIdx*3]]
                
                # calculate closest solution for next position
                goal_ths  = A1_kinematics.calc_correct_thetas([self.hip_to_toe_pos[legIdx][0] + self.x_shift, 
                                                               self.hip_to_toe_pos[legIdx][1], 
                                                               self.hip_to_toe_pos[legIdx][2]], current_ths, legIdx % 2 == 0)
                
                # set goal angles for corresponding leg
                self.goal_pos[legIdx*3] = goal_ths[2]
                self.goal_pos[legIdx*3 + 1] = goal_ths[0]
                self.goal_pos[legIdx*3 + 2] = goal_ths[1] + np.pi/2     
                         
            for i in range(0, len(joint_names)):
                motor_command.q = self.goal_pos[i]
                self.publishers[i].publish(motor_command)  

            t+=1
            t %= step_period

            self.rate.sleep()
         
    
        
    def joint_states_callback(self, data):
        self.positions = data.position
        self.velocities = data.velocity
    
    def imu_callback(self, data):
        quaternion = [data.orientation.w,
                      data.orientation.x, 
                      data.orientation.y, 
                      data.orientation.z]
        
        euler_orientation = transformations.euler_from_quaternion(quaternion)
        
        self.current_roll = euler_orientation[2]
        self.current_pitch = euler_orientation[1]
        self.current_yaw = euler_orientation[0] + np.pi
        
        
        self.pitch = -self.current_pitch/100
        self.roll = self.current_roll/100
        #self.yaw = -self.current_yaw
        #print(f"Angles: \n {euler_orientation} \n_________________________________________________________")


if __name__ == '__main__':

    try:
        effort_publisher = A1Controller()
        effort_publisher.publish_commands()
    except rospy.ROSInterruptException:
        pass