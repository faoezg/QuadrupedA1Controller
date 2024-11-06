"""receives joint positions from the high-level controller and sends corresponding torque commands according to PD control"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.wait_for_message import wait_for_message
from std_msgs.msg import Float64

command_topics = ["/FL_hip_cmd", "/FL_thigh_cmd", "/FL_calf_cmd",
                  "/FR_hip_cmd", "/FR_thigh_cmd", "/FR_calf_cmd",
                  "/RL_hip_cmd", "/RL_thigh_cmd", "/RL_calf_cmd",
                  "/RR_hip_cmd", "/RR_thigh_cmd", "/RR_calf_cmd"]

class LLController(Node):
    def __init__(self):
        super().__init__('a1_controller')
        self.freq = 500
        self.timer = self.create_timer(1.0 / self.freq, self.update)
    
        # subscribe to joint commands
        self.create_subscription(JointState, "/high_level/joint_cmd", self.joint_cmd_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)

        # receive first message to set the starting position
        _,msg = wait_for_message(msg_type=JointState, node=self,topic="/joint_states",time_to_wait=5)
        self.velocities = msg.velocity
        self.positions = msg.position

        # publishers for the joint commands
        self.pubs = []
        for topic in command_topics:
            self.pubs.append(self.create_publisher(Float64, topic, 10))  # Create Publisher for each Joint

        self.desired_thetas = self.positions
        self.cmd_thetas = [0.0]*12

        # PID Controller Parameters
        self.kp = [175.0]*12
        self.kp[0] = 50.0
        self.kp[3] = 50.0
        self.kp[6] = 50.0
        self.kp[9] = 50.0 
        print(self.kp)
        self.kd = [2.0]*12



    """Main loop of the controller, updates at self.freq"""
    def update(self):

        for i in range(len(self.pubs)):
            # PD Controller
            self.cmd_thetas[i] = self.kp[i] * (self.desired_thetas[i] - self.positions[i]) - self.kd[i] * self.velocities[i]

            motor_cmd = Float64()
            motor_cmd.data = self.cmd_thetas[i]
            self.pubs[i].publish(motor_cmd)
    

    def joint_cmd_callback(self, msg):
        self.desired_thetas = msg.position
        
    def joint_states_callback(self, msg):
        self.positions = msg.position
        self.velocities = msg.velocity
    

def main(args=None):
    rclpy.init(args=args)
    controller = LLController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

