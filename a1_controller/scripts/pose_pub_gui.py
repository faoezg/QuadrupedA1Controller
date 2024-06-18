#!/usr/bin/env python3


import pygame
import numpy as np
import rospy
from geometry_msgs.msg import Pose

class PosePubGUI:
    def __init__(self, width=400, height=300):
        pygame.init()
        pygame.font.init()
        
        self.FONT_SIZE = 24
        self.font = pygame.font.SysFont(None, self.FONT_SIZE)
        
        
        # Screen dimensions and setup
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Pose Pub 3D")

        # Constants
        self.SLIDER_RADIUS = 50
        self.JOYSTICK_RADIUS = 15
        self.CENTER1 = (width*1/5, height/2)
        self.CENTER2 = (width/2, height/2)
        self.CENTER3 = (width*4/5, height/2)


        # Joystick positions
        self.joystick1_pos = self.CENTER1
        self.joystick2_pos = self.CENTER2
        self.joystick3_pos = self.CENTER3
        
        # Active joystick flags
        self.active_joystick1 = False
        self.active_joystick2 = False
        self.active_joystick3 = False
        
        # init ROS and Robots Positions:
        rospy.init_node('pose_publisher_gui', anonymous=True)
        self.pose_publisher = rospy.Publisher("/goal_pose", Pose, queue_size=1)
        self.rate = rospy.Rate(25)
        
        self.slider_height = self.base_height = 0.225
        self.slider_width  = self.base_width = -0.0838
        self.slider_length = self.base_length = 0.0
        
        self.slider_yaw = self.yaw = 0.0       
        self.slider_pitch = self.pitch = 0.0  
        self.slider_roll = self.roll = 0.0 
            
    def draw_joystick(self, center, position, color, label = "No Label"):
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
                elif np.hypot(event.pos[0] - self.joystick3_pos[0], event.pos[1] - self.joystick3_pos[1]) <= self.JOYSTICK_RADIUS:
                    self.active_joystick3 = True
                    
            elif event.type == pygame.MOUSEBUTTONUP:
                self.active_joystick1 = False
                self.active_joystick2 = False
                self.active_joystick3 = False
                # reset if slider is let go 
                self.slider_pitch = self.slider_roll = self.slider_yaw = 0.0
                self.slider_height = self.base_height
                self.slider_width = self.base_width
                self.slider_length = 0.0
                self.joystick1_pos = self.CENTER1
                self.joystick2_pos = self.CENTER2
                self.joystick3_pos = self.CENTER3
                
            elif event.type == pygame.MOUSEMOTION:
                if self.active_joystick1:
                    self.joystick1_pos = self.get_joystick_position(self.CENTER1, event.pos)
                    self.slider_roll = ((self.joystick1_pos[0] - self.CENTER1[0]) / (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS))/2
                    self.slider_length = ((self.joystick1_pos[1] - self.CENTER1[1]) / (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS))/20
                    
                elif self.active_joystick2:
                    self.joystick2_pos = self.get_joystick_position(self.CENTER2, event.pos)
                    self.slider_yaw = ((self.joystick2_pos[0] - self.CENTER2[0]) / (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS))/2
                    self.slider_height = self.base_height - ((self.joystick2_pos[1] - self.CENTER2[1]) / (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS))/10    
                                 
                elif self.active_joystick3:
                    self.joystick3_pos = self.get_joystick_position(self.CENTER3, event.pos)
                    self.slider_pitch = ((self.joystick3_pos[0] - self.CENTER3[0]) / (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS))/2
                    self.slider_width = self.base_width - ((self.joystick3_pos[1] - self.CENTER3[1]) / (self.SLIDER_RADIUS - self.JOYSTICK_RADIUS))/10
        return True

    def run(self):
        running = True
        
        # init Pose3D Message:
        pose_msg = Pose()

        ## initialize display:
        self.screen.fill((255, 255, 255))
        self.draw_joystick(self.CENTER1, self.joystick1_pos, (0, 0, 255), "Roll/Move X")
        self.draw_joystick(self.CENTER2, self.joystick2_pos, (255, 0, 0), "Yaw/Move Z")
        self.draw_joystick(self.CENTER3, self.joystick3_pos, (0, 255, 0), "Pitch/Move Y")
        pygame.display.flip()
        
        while running:
            running = self.handle_events()
            
            
            # set up message & publish
            pose_msg.position.x = self.slider_width
            pose_msg.position.y = self.slider_height
            pose_msg.position.z = self.slider_length
            pose_msg.orientation.x = self.slider_roll  # These are all euler angles because -
            pose_msg.orientation.y = self.slider_pitch  # converting back and forth leaves annoying - 
            pose_msg.orientation.z = self.slider_yaw  # edge cases 
            pose_msg.orientation.w = 0
            self.pose_publisher.publish(pose_msg)
             
                
            self.screen.fill((255, 255, 255))

            # Draw joysticks
            self.draw_joystick(self.CENTER1, self.joystick1_pos, (0, 0, 255), "Roll/Move X")
            self.draw_joystick(self.CENTER2, self.joystick2_pos, (255, 0, 0), "Yaw/Move Z")
            self.draw_joystick(self.CENTER3, self.joystick3_pos, (0, 255, 0), "Pitch/Move Y")
            
            # Update display
            pygame.display.flip()
            self.rate.sleep()
            
        pygame.quit()


if __name__ == "__main__":
    controller = PosePubGUI()
    controller.run()
