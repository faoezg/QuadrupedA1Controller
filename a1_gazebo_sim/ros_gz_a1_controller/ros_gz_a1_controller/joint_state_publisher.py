#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.wait_for_message import wait_for_message
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from .lib import A1_kinematics
from .lib.Trajectory_Planner import Trajectory_Planner
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


class PoseController(Node):
    def __init__(self, width=400, height=300):
        super().__init__('move_publisher')

        # init ROS and Robots Positions:
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Pose, "/goal_pose", self.goal_pos_callback, 10)
        self.freq = 500
        self.timer = self.create_timer(1.0 / self.freq, self.update)

        self.positions = []

        # standing configuration
        self.goal_pos = np.array([-1.9583591983757351, -0.0007974578255129927, 0.9794434592400876,
                                  -1.9580158278760527, 0.00048751519737599835, 0.97896869674112,
                                  -1.968766039552742, 0.0005508150816577739, 0.9651295186701967,
                                  -1.968942195136563, 0.0002753686771956865, 0.9639652783917043])

        # receive first message to set the starting position
        _,first_pos = wait_for_message(msg_type=JointState, node=self,topic="/joint_states",time_to_wait=5)
        self.startup_pos = np.zeros(12)

        # reformat the message to match the order of the goal_pos
        for k in range(0,4):
            self.startup_pos[0 + k*3] = first_pos.position[2 + k*3]
            self.startup_pos[1 + k*3] = first_pos.position[0 + k*3]
            self.startup_pos[2 + k*3] = first_pos.position[1 + k*3]

        self.base_height = 0.225
        self.base_width = -0.0838
        self.hip_to_toe_pos = [[-0.0838, 0.225, 0.0],  # FL
                               [0.0838, 0.225, 0.0],  # FR
                               [-0.0838, 0.225, 0.0],  # RL
                               [0.0838, 0.225, 0.0]]  # RR

        self.goal_height = self.height = self.hip_to_toe_pos[0][1]
        self.goal_width = self.width = self.hip_to_toe_pos[0][0]
        self.goal_length = self.length = self.hip_to_toe_pos[0][2]

        self.goal_yaw = self.yaw = 0.0
        self.goal_pitch = self.pitch = 0.0
        self.goal_roll = self.roll = 0.0

        self.global_positions = [[0, 0, 0],
                                 [0, 0, 0],
                                 [0, 0, 0],
                                 [0, 0, 0]]
        self.pubs = []
        for topic in command_topics:
            self.pubs.append(self.create_publisher(Float64, topic, 10))  # Create Publisher for each Joint

        self.motor_command = Float64()
        self.tp = Trajectory_Planner()

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

        print("Listening to /goal_pose topic. Use the pose_pub_gui node or RQT to publish Pose messages.")

    def update(self):
        # calculate error for each joystick value to current value
        roll_error = (self.goal_roll - self.roll) / 100
        yaw_error = (self.goal_yaw - self.yaw) / 100  # dividing to smoothen the movement
        pitch_error = (self.goal_pitch - self.pitch) / 100
        height_error = (self.goal_height - self.height) / 100
        length_error = (self.goal_length - self.length) / 100
        width_error = (self.goal_width - self.width) / 100

        # ROS CONTROL LOOP
        for legIdx in range(4):
            # add translation
            self.hip_to_toe_pos[legIdx][0] += width_error
            self.hip_to_toe_pos[legIdx][1] += height_error
            self.hip_to_toe_pos[legIdx][2] += length_error

            # calculate global positions (base to foot)
            self.global_positions[legIdx] = self.tp.global_foot_pos(legIdx, self.hip_to_toe_pos[legIdx])

            # apply RPY via rotation matrix
            self.global_positions[legIdx] = self.tp.apply_rpy(self.global_positions[legIdx][0],
                                                              self.global_positions[legIdx][1],
                                                              self.global_positions[legIdx][2],
                                                              roll_error, pitch_error, yaw_error)

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

        # update the overall amount for each angle
        self.yaw += yaw_error
        self.pitch += pitch_error
        self.roll += roll_error
        self.height += height_error
        self.length += length_error
        self.width += width_error

        # send joint commands
        for i in range(len(self.pubs)):
            self.motor_command.data = self.goal_pos[i]
            self.pubs[i].publish(self.motor_command)

    def joint_states_callback(self, msg):
        self.positions = msg.position

    def goal_pos_callback(self, msg):
        self.goal_width = msg.position.x
        self.goal_height = msg.position.y
        self.goal_length = msg.position.z

        self.goal_roll = msg.orientation.x
        self.goal_pitch = msg.orientation.y
        self.goal_yaw = msg.orientation.z

def main(args=None):
    rclpy.init(args=args)
    controller = PoseController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
