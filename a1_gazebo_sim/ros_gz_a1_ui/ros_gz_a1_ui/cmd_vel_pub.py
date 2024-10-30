#!/usr/bin/env python3

import pygame
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

class CMDVelPubGUI(Node):
    def __init__(self, width=400, height=300):
        super().__init__('cmdvel_pub_gui')
        
        pygame.init()
        pygame.font.init()
        
        self.FONT_SIZE = 24
        self.font = pygame.font.SysFont(None, self.FONT_SIZE)
        self.running = True
        
        # Screen dimensions and setup
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("CMDVel Pub 3D")

        # Constants
        self.SLIDER_RADIUS = 50
        self.JOYSTICK_RADIUS = 15
        self.CENTER1 = (width*1/4, height/2)
        self.CENTER2 = (width*3/4, height/2)
        
        # Joystick positions
        self.joystick1_pos = self.CENTER1
        self.joystick2_pos = self.CENTER2
        
        # Active joystick flags
        self.active_joystick1 = False
        self.active_joystick2 = False
        
        # ROS2 setup
        self.pose_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.04, self.publish_and_update)  # 25 Hz

        self.slider_x_vel, self.slider_y_vel, self.slider_yaw_vel = 0.0       


    def draw_joystick(self, center, position, color, label="No Label"):
        pygame.draw.circle(self.screen, color, center, self.SLIDER_RADIUS, 2)
        pygame.draw.circle(self.screen, color, position, self.JOYSTICK_RADIUS)
        text_surface = self.font.render(label, True, color)
        text_rect = text_surface.get_rect(center=(center[0], center[1] - self.SLIDER_RADIUS - 20))
        self.screen.blit(text_surface, text_rect)

    def get_joystick_position(self, center, mouse_pos):
        dx = mouse_pos[0] - center[0]
        dy = mouse_pos[1] - center[1]
        distance = np.hypot(dx, dy) 
        if distance > self.SLIDER_RADIUS - self.JOYSTICK_RADIUS:
            angle = np.arctan2(dy, dx) 
            dx = (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS) * np.cos(angle)
            dy = (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS) * np.sin(angle)
        return (center[0] + dx, center[1] + dy)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if np.hypot(event.pos[0] - self.joystick1_pos[0], event.pos[1] - self.joystick1_pos[1]) <= self.JOYSTICK_RADIUS:
                    self.active_joystick1 = True

                elif np.hypot(event.pos[0] - self.joystick2_pos[0], event.pos[1] - self.joystick2_pos[1]) <= self.JOYSTICK_RADIUS:
                    self.active_joystick2 = True
                    
            elif event.type == pygame.MOUSEBUTTONUP:
                self.active_joystick1 = False
                self.active_joystick2 = False
                # reset if slider is let go 
                
                self.slider_x_vel, self.slider_y_vel, self.slider_yaw_vel= 0.0
                self.joystick1_pos = self.CENTER1
                self.joystick2_pos = self.CENTER2
                
            elif event.type == pygame.MOUSEMOTION:
                if self.active_joystick1:
                    self.joystick1_pos = self.get_joystick_position(self.CENTER1, event.pos)
                    self.slider_y_vel = ((self.joystick1_pos[0] - self.CENTER1[0]) / (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS))
                    self.slider_x_vel = ((self.joystick1_pos[1] - self.CENTER1[1]) / (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS))
                                 
                elif self.active_joystick2:
                    self.joystick2_pos = self.get_joystick_position(self.CENTER2, event.pos)
                    self.slider_yaw_vel = ((self.joystick2_pos[0] - self.CENTER2[0]) / (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS))
        return True

    def publish_and_update(self): 
        cmd_vel_msg = Twist()
            

        cmd_vel_msg.linear.x = self.slider_x_vel
        cmd_vel_msg.linear.y = self.slider_y_vel
        
        cmd_vel_msg.angular.z = self.slider_yaw_vel

        self.pose_publisher.publish(cmd_vel_msg)
        
        self.running = self.handle_events()
        self.screen.fill((255, 255, 255))

        # Draw joysticks
        self.draw_joystick(self.CENTER1, self.joystick1_pos, (0, 0, 255), "Linear Velocity")
        self.draw_joystick(self.CENTER2, self.joystick2_pos, (0, 255, 0), "Angular Velocity")
                
        # Update display
        pygame.display.flip()
 

def main(args=None):
    rclpy.init(args=args)
    controller = CMDVelPubGUI()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
