"""This script controls the Quadruped A1 in gazebo launched in use_force_ctrl-Mode"""
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Quaternion, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from .lib.Trajectory_Planner import Trajectory_Planner
from .lib import A1_kinematics

command_topics = ["/FL_hip_cmd", "/FL_thigh_cmd", "/FL_calf_cmd",
                  "/FR_hip_cmd", "/FR_thigh_cmd", "/FR_calf_cmd",
                  "/RL_hip_cmd", "/RL_thigh_cmd", "/RL_calf_cmd",
                  "/RR_hip_cmd", "/RR_thigh_cmd", "/RR_calf_cmd"]

class A1Controller(Node):
    def __init__(self):
        super().__init__('a1_controller')
        self.freq = 100
        self.timer = self.create_timer(1.0 / self.freq, self.update)
    
        # subscribe to the available sensor data
        #self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.create_subscription(Contacts, "/forces/FL_contact_force", self.FL_contact_callback, 10)
        self.create_subscription(Contacts, "/forces/FR_contact_force", self.FR_contact_callback, 10)
        self.create_subscription(Contacts, "/forces/RL_contact_force", self.RL_contact_callback, 10)
        self.create_subscription(Contacts, "/forces/RR_contact_force", self.RR_contact_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, "/a1_ign/odometry", self.odom_callback, 10)

        # Initialize State Variables
        self.orientation = Quaternion()
        self.angular_vel = Vector3()
        self.linear_acc = Vector3()
        self.positions = [0.0]*12
        self.velocities = [0.0]*12

        self.contacts = [[0, 0],
                         [0, 0],
                         [0, 0],
                         [0, 0]]  # [force, contact (1 or 0)]
        

        self.desired_theta = [0.,1.,-2.,
                              -0.,1.,-2.,
                              0.,1.,-2.,
                              -0.,1.,-2.] # initial standing position FL,FR,RL,RR
        
        #self.cmd_thetas = [0.0]*12
        self.pub = self.create_publisher(JointState, "/high_level/joint_cmd", 10)
        self.msg = JointState()
        """self.msg.name = ["FL_hip", "FL_thigh", "FL_calf",
                        "FR_hip", "FR_thigh", "FR_calf",
                        "RL_hip", "RL_thigh", "RL_calf",
                        "RR_hip", "RR_thigh", "RR_calf"]"""
        
        self.hip_to_toe_pos = [[-0.0838, 0.225, 0.0],  # FL
                               [0.0838, 0.225, 0.0],  # FR
                               [-0.0838, 0.225, 0.0],  # RL
                               [0.0838, 0.225, 0.0]]  # RR

        self.default_stance = np.array([0,1,-2,
                               0,1,-2,
                               0,1,-2,
                               0,1,-2])
        
        self.laying_down = np.array([0.0,2.69,-2.69,
                               0.0,2.69,-2.69,
                               0.0,2.69,-2.69,
                               0.0,2.69,-2.69])
        self.x_shift = 0.06
        self.goal_yaw = self.yaw = self.pitch = self.roll = 0.0
        self.linear_vel = [0.0, 0.0]
        self.linear_cmd_vel = [0.0, 0.0]

        self.global_positions = [[0, 0, 0],
                                 [0, 0, 0],
                                 [0, 0, 0],
                                 [0, 0, 0]]
        
        self.tp = Trajectory_Planner()
        self.t = 0

        self.stand_up()
        pass

    """Main loop of the controller, updates at self.freq"""
    def update(self):
        
        for legIdx in range(0,4):

            # calculate next step:
            self.hip_to_toe_pos[legIdx] = self.tp.trot(legIdx, self.hip_to_toe_pos[legIdx], 0.05, 50, self.t, self.linear_vel, self.linear_cmd_vel)
            # print(self.hip_to_toe_pos[legIdx])
            #print(self.t)
            # calculate global positions (base to foot)
            self.global_positions[legIdx] = self.tp.global_foot_pos(legIdx, self.hip_to_toe_pos[legIdx])
            
            # apply RPY via rotation matrix
            self.global_positions[legIdx] = self.tp.apply_rpy(self.global_positions[legIdx][0], 
                                                      self.global_positions[legIdx][1], 
                                                      self.global_positions[legIdx][2], 
                                                      self.roll, self.pitch, self.yaw)
            
            # set new local position (hip to foot)
            self.hip_to_toe_pos[legIdx] = self.tp.local_foot_pos(legIdx,self.global_positions[legIdx])
            
            # get current leg angles from robot
            current_ths = [self.positions[legIdx*3 + 0], 
                           self.positions[legIdx*3 + 1], 
                           self.positions[legIdx*3 + 2]]
            
            # calculate closest solution for next position
            goal_ths  = A1_kinematics.calc_correct_thetas([self.hip_to_toe_pos[legIdx][0], 
                                                           self.hip_to_toe_pos[legIdx][1], 
                                                           self.hip_to_toe_pos[legIdx][2]], current_ths, legIdx % 2 == 0)
            
            # set goal angles for corresponding leg
            self.desired_theta[legIdx*3 + 2] = goal_ths[2]
            self.desired_theta[legIdx*3] = goal_ths[0]
            self.desired_theta[legIdx*3 + 1] = goal_ths[1]     


        
        self.msg.position = self.desired_theta
        self.pub.publish(self.msg)

        self.t+=1
        self.t %= 50
        
    """Start Up Routine"""    
    def stand_up(self):
        print("Standing up")
        num_steps = self.freq  # one sec to stand up at the start
        step = (self.default_stance - self.laying_down) / num_steps
        for _ in range(num_steps):
            self.laying_down += step
            self.msg.position = self.laying_down
            self.pub.publish(self.msg)
            time.sleep(1.0/self.freq)

        time.sleep(1.0)

    """Callback for the sensor data"""
    #def joint_states_callback(self, msg):
    #    self.positions = msg.position
    #    self.velocities = msg.velocity

    def imu_callback(self, msg):
        self.orientation = msg.orientation
        self.angular_vel = msg.angular_velocity
        self.linear_acc = msg.linear_acceleration

        self.acc_x = msg.linear_acceleration.x
        self.acc_y = msg.linear_acceleration.y
        self.acc_z = msg.linear_acceleration.z

    def FL_contact_callback(self, msg):
        z_force = msg.contacts[0].wrenches[0].body_1_wrench.force.z
        self.contacts[0] = [z_force, 1 if z_force > 0 else 0]
    
    def FR_contact_callback(self, msg):
        z_force = msg.contacts[0].wrenches[0].body_1_wrench.force.z
        self.contacts[1] = [z_force, 1 if z_force > 0 else 0]
            
    def RL_contact_callback(self, msg):
        z_force = msg.contacts[0].wrenches[0].body_1_wrench.force.z
        self.contacts[2] = [z_force, 1 if z_force > 0 else 0]
    
    def RR_contact_callback(self, msg):
        z_force = msg.contacts[0].wrenches[0].body_1_wrench.force.z
        self.contacts[3] = [z_force, 1 if z_force > 0 else 0]

    def cmd_vel_callback(self, msg):
        self.linear_cmd_vel = [msg.linear.x, msg.linear.y]
        self.angular_vel = msg.angular

    def odom_callback(self, msg):
        self.linear_vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]
    

def main(args=None):
    rclpy.init(args=args)
    controller = A1Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

