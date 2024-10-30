"""This script controls the Quadruped A1 in gazebo launched in use_force_ctrl-Mode"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float64


command_topics = ["/FL_hip_cmd", "/FL_thigh_cmd", "/FL_calf_cmd",
                  "/FR_hip_cmd", "/FR_thigh_cmd", "/FR_calf_cmd",
                  "/RL_hip_cmd", "/RL_thigh_cmd", "/RL_calf_cmd",
                  "/RR_hip_cmd", "/RR_thigh_cmd", "/RR_calf_cmd"]

class A1Controller(Node):
    def __init__(self):
        super().__init__('a1_controller')
        self.freq = 500
        self.timer = self.create_timer(1.0 / self.freq, self.update)
    
        # subscribe to the available sensor data
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.create_subscription(Contacts, "/forces/FL_contact_force", self.FL_contact_callback, 10)
        self.create_subscription(Contacts, "/forces/FR_contact_force", self.FR_contact_callback, 10)
        self.create_subscription(Contacts, "/forces/RL_contact_force", self.RL_contact_callback, 10)
        self.create_subscription(Contacts, "/forces/RR_contact_force", self.RR_contact_callback, 10)


        self.contacts = [[0, 0],
                         [0, 0],
                         [0, 0],
                         [0, 0]]  # [force, contact (1 or 0)]
        
        # publishers for the joint commands

        self.pubs = []
        for topic in command_topics:
            self.pubs.append(self.create_publisher(Float64, topic, 10))  # Create Publisher for each Joint



        pass

    """Main loop of the controller, updates at self.freq"""
    def update(self):
        pass
    










    """Callback for the sensor data"""
    def joint_states_callback(self, msg):
        self.positions = msg.position
        self.velocities = msg.velocity
        print(self.positions)

    def imu_callback(self, msg):
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


def main(args=None):
    rclpy.init(args=args)
    controller = A1Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

