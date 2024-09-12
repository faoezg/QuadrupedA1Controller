#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.wait_for_message import wait_for_message
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from ros_gz_interfaces.msg import Contacts
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from . import A1_kinematics
from .Trajectory_Planner import Trajectory_Planner
import time


command_topics = ["/FL_calf_cmd",
                  "/FL_hip_cmd",
                  "/FL_thigh_cmd",
                  "/FR_calf_cmd",
                  "/FR_hip_cmd",
                  "/FR_thigh_cmd",
                  "/RL_calf_cmd",
                  "/RL_hip_cmd",
                  "/RL_thigh_cmd",
                  "/RR_calf_cmd",
                  "/RR_hip_cmd",
                  "/RR_thigh_cmd"]


class StandController(Node):
    def __init__(self, width=400, height=300):
        super().__init__('move_publisher')

        # init ROS and Robots Positions:
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.create_subscription(Contacts, "/forces/FL_contact_force", self.FL_contact_callback, 10)
        self.create_subscription(Contacts, "/forces/FR_contact_force", self.FR_contact_callback, 10)
        self.create_subscription(Contacts, "/forces/RL_contact_force", self.RL_contact_callback, 10)
        self.create_subscription(Contacts, "/forces/RR_contact_force", self.RR_contact_callback, 10)

        self.freq = 500
        self.timer = self.create_timer(1.0 / self.freq, self.update)

        self.positions = []

        # standing configuration
        self.goal_pos = np.array([-1.6333181007654698, 0.0007974578255129927, 0.8246591003220253,
                                  -1.6333181007654698, 0.00048751519737599835, 0.8246591003220253,
                                  -1.6333181007654698, 0.0005508150816577739, 0.8246591003220253,
                                  -1.6333181007654698, 0.0002753686771956865, 0.8246591003220253])

        # receive first message to set the starting position
        _,first_pos = wait_for_message(msg_type=JointState, node=self,topic="/joint_states",time_to_wait=5)
        
        print(first_pos)
        self.startup_pos = np.array(first_pos.position)

        self.base_height = 0.225
        self.base_width = -0.0838
        self.hip_to_toe_pos = [[-0.0838, 0.275, 0.0],  # FL
                               [0.0838, 0.275, 0.0],  # FR
                               [-0.0838, 0.275, 0.0],  # RL
                               [0.0838, 0.275, 0.0]]  # RR

        self.height = self.hip_to_toe_pos[0][1]
        self.width = self.hip_to_toe_pos[0][0]
        self.length = self.hip_to_toe_pos[0][2]

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.global_positions = [[0, 0, 0],
                                 [0, 0, 0],
                                 [0, 0, 0],
                                 [0, 0, 0]]
        
        self.acc_x = self.acc_y = self.acc_z = 0
        self.contacts = [[0, 0],
                         [0, 0],
                         [0, 0],
                         [0, 0]]  # [force, contact (1 or 0)]

        self.pubs = []
        for topic in command_topics:
            self.pubs.append(self.create_publisher(Float64, topic, 10))  # Create Publisher for each Joint

        self.motor_command = Float64()
        self.tp = Trajectory_Planner()
        self.t = 0
        self.stand_up()
        self.positions = self.goal_pos


    def stand_up(self):
        print("Standing up")
        num_steps = self.freq  # one sec to stand up at the start
        step = (self.goal_pos - self.startup_pos) / num_steps

        for i in range(num_steps):
            for i in range(len(self.pubs)):
                self.startup_pos[i] += step[i]
                self.motor_command.data = self.startup_pos[i]
                self.pubs[i].publish(self.motor_command)
            time.sleep(1.0/self.freq)

            
    ## ROS CONTROL LOOP
    def update(self):

        for legIdx in range(4):

            # calculate next step:
            self.hip_to_toe_pos[legIdx] = self.tp.step_on_spot(legIdx, self.hip_to_toe_pos[legIdx], 0.10, self.freq*4, self.t)
            # calculate global positions (base to foot)
            self.global_positions[legIdx] = self.tp.global_foot_pos(legIdx, self.hip_to_toe_pos[legIdx])

            # apply RPY via rotation matrix
            self.global_positions[legIdx] = self.tp.apply_rpy(self.global_positions[legIdx][0],
                                                              self.global_positions[legIdx][1],
                                                              self.global_positions[legIdx][2],
                                                              0,0,0)
            
            # set new local position (hip to foot)
            self.hip_to_toe_pos[legIdx] = self.tp.local_foot_pos(legIdx, self.global_positions[legIdx])

            # get current leg angles from robot
            current_ths = [self.positions[legIdx * 3 + 1],
                           self.positions[legIdx * 3 + 2],
                           self.positions[legIdx * 3]]

            # calculate closest solution for next position
            goal_ths = A1_kinematics.calc_correct_thetas([self.hip_to_toe_pos[legIdx][0],
                                                          self.hip_to_toe_pos[legIdx][1],
                                                          self.hip_to_toe_pos[legIdx][2]], current_ths, legIdx % 2 == 0)

            # set goal angles for corresponding leg
            self.goal_pos[legIdx * 3] = goal_ths[2]
            self.goal_pos[legIdx * 3 + 1] = goal_ths[0]
            self.goal_pos[legIdx * 3 + 2] = goal_ths[1]

        # send joint commands
        for i in range(len(self.pubs)):
            self.motor_command.data = self.goal_pos[i]
            self.pubs[i].publish(self.motor_command)

        self.t+=1
        self.t %= self.freq*4

    def joint_states_callback(self, msg):
        self.positions = msg.position

    def imu_callback(self, msg):
        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z

    def FL_contact_callback(self, msg):
        z_force = msg.contacts[0].wrenches[0].body_1_wrench.force.z
        self.contacts[0] = [z_force, 1 if z_force > 0 else 0]
        print(z_force)
    
    def FR_contact_callback(self, msg):
        z_force = msg.contacts[0].wrenches[0].body_1_wrench.force.z
        self.contacts[1] = [z_force, 1 if z_force > 0 else 0]
            
    def RL_contact_callback(self, msg):
        z_force = msg.contacts[0].wrenches[0].body_1_wrench.force.z
        self.contacts[2] = [z_force, 1 if z_force > 0 else 0]
    
    def RR_contact_callback(self, msg):
        z_force = msg.contacts[0].wrenches[0].body_1_wrench.force.z
        self.contacts[3] = [z_force, 1 if z_force > 0 else 0]
    

def main(args=None):
    rclpy.init(args=args)
    controller = StandController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
