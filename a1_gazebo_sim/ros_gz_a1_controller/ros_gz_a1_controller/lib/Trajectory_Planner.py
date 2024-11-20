import numpy as np
from .bezier import bezier_curve, get_control_points

class Trajectory_Planner:  
    """Class used to bundle trajectory generation and other helpful
                functions related to the foot positions""" 
                 
    def __init__(self, base_height=0.225, base_width=0.0838) -> None:
        self.leg_offset_y = 0.1805
        self.leg_offset_x = 0.047
        self.base_height = base_height
        self.base_width = base_width
        self.z_fd = 0.0
        self.x_fd = 0.0

    def big_steppa(self, legIdx,position, step_height, step_length, T_period, t):
        # -positioon: current x,y,z coordinates
        # -step height: adjusts movement in Y direction (up/down)
        # -step_length: adjusts movement in Z direction (forward/backward)
        # -T_period: duration of one step
        # -T_stand: duration of stand phase
        # -t: current time
        x = position[0]
        y = position[1]
        z = position[2]
        T_swing = 1/6 * T_period
        T_stand = 3/6 * T_period
        global_t = t
        
        if legIdx == 2:
            t += 1/6 * T_period
            t %= T_period
            
        if legIdx == 0:
            t += 2/6* T_period
            t %= T_period
        
        if legIdx == 3:
            t += 4/6 * T_period
            t %= T_period 
            
        if legIdx == 1:
            t += 5/6 * T_period
            t %= T_period 
                         
        if 3/6 * T_period <= global_t < 4/6*T_period:
            x += 1.5*0.0048  
        elif global_t < 1/6*T_period:
            x -= 1.5*0.0048
            
        if t < T_stand:
                # no movement up, only back
            z = t*(step_length)/(T_stand) - step_length/2 + 0.05
                
            
        if T_stand <= t < T_stand + T_swing:
            u = t - T_stand
            # movement in y direction according to sin, + movement forward
            y = -step_height * np.sin(np.pi*u/(T_swing-1)) + self.base_height
            z = (step_length) - u*(step_length)/T_swing - step_length/2 + 0.05
                
        return [x, y, z]
    

    def trot_bezier(self, legIdx, position, T_period, t, command_vel=[0,0], angular_command_vel=0):
        x = position[0]   
        y = position[1]   
        z = position[2]   
        desired_vel_z = command_vel[0]/2  # FORWARD VELOCITY (negative because negative z axis in hip coordinates is forward in world coordinates)  
        desired_vel_x = command_vel[1]/2  # LATERAL VELOCITY

        if legIdx in (0, 3):  # For legs 0 and 3, adjust time offset for trotting
            t += T_period / 2
        t %= T_period

        if t==0 or t== T_period/2:  # limit updating of control points for less eradic behaviours
            self.control_points_fw = get_control_points(desired_vel_z*1000, 1/2)  # control points for forward movement
            self.control_points_lt = get_control_points(desired_vel_x*1000, 1/2)  # for lateral movement
            self.control_points_yaw = get_control_points(angular_command_vel*1000, 1/2)  # for yaw movement
        
            self.start_fw,_ = bezier_curve(0, self.control_points_fw)
            self.end_fw,_ = bezier_curve(1, self.control_points_fw)
            self.start_lt,_ = bezier_curve(0, self.control_points_lt)
            self.end_lt,_ = bezier_curve(1, self.control_points_lt)
            self.start_yaw,_ = bezier_curve(0, self.control_points_yaw)
            self.end_yaw,_ = bezier_curve(1, self.control_points_yaw)
            

            self.start_end_dist_fw = abs(self.start_fw) + abs(self.end_fw)
            self.start_end_dist_lt = abs(self.start_lt) + abs(self.end_lt)
            self.start_end_dist_yaw = abs(self.start_yaw) + abs(self.end_yaw)


        if t <= T_period / 2:  # Swing phase
            z_vals, y_vals_z = bezier_curve(t/(T_period/2), self.control_points_fw)
            x_vals, y_vals_x = bezier_curve(t/(T_period/2), self.control_points_lt)
            yaw_vals, y_vals_yaw = bezier_curve(t/(T_period/2), self.control_points_yaw)

            if angular_command_vel == 0:
                y = self.base_height - y_vals_z/1000 if np.abs(desired_vel_z) > np.abs(desired_vel_x) else self.base_height - y_vals_x/1000

            z = -(z_vals/3000) 

            x = -(x_vals/6000)- (self.base_width) if legIdx % 2 == 0 else -(x_vals/6000) + self.base_width

            if angular_command_vel != 0:
                y = self.base_height - y_vals_yaw/1000
                # calculte position in body frame:
                pos = self.global_foot_pos(legIdx, [x,y,z])

                # apply yaw rotation
                pos = self.apply_rpy(pos[0], pos[1], pos[2], 0, 0, yaw_vals/6000)

                # convert back to hip frame

                x, y, z = self.local_foot_pos(legIdx, pos)


        else: # Stand phase

            if desired_vel_z < 0:
                     z -= self.start_end_dist_fw/(3000 * T_period/2)
            elif desired_vel_z >=0:
                     z += self.start_end_dist_fw/(3000 * T_period/2)
            if desired_vel_x < 0:
                x -= self.start_end_dist_lt/(6000 * T_period/2)
            elif desired_vel_x >= 0:
                x += self.start_end_dist_lt/(6000 * T_period/2)

            # rotate back the amount rotated during swing phase
            if angular_command_vel != 0:
                if angular_command_vel > 0:
                    pos = self.global_foot_pos(legIdx, [x,y,z])
                    pos = self.apply_rpy(pos[0], pos[1], pos[2], 0, 0, -(self.start_end_dist_yaw)/(6000 * T_period/2))
                    x, y, z = self.local_foot_pos(legIdx, pos)
                elif angular_command_vel < 0:
                    pos = self.global_foot_pos(legIdx, [x,y,z])
                    pos = self.apply_rpy(pos[0], pos[1], pos[2], 0, 0, (self.start_end_dist_yaw)/(6000 * T_period/2))
                    x, y, z = self.local_foot_pos(legIdx, pos)    
                    
        return [x, y, z]

    
    def global_foot_pos(self, id, position):  # calculates the foot position in reference to the base link of the quadruped

        if id == 0:  # FL
            position[0] -= self.leg_offset_x
            position[2] -= self.leg_offset_y
        
        elif id == 1:  # FR
            position[0] += self.leg_offset_x
            position[2] -= self.leg_offset_y
        
        elif id == 2:  # RL
            position[0] -= self.leg_offset_x
            position[2] += self.leg_offset_y
        
        elif id == 3:  # RR
            position[0] += self.leg_offset_x
            position[2] += self.leg_offset_y

        return position

    def local_foot_pos(self, id, position):  # does the inverse to the above function
        if id == 0:  # FL
            position[0] += self.leg_offset_x
            position[2] += self.leg_offset_y
        
        elif id == 1:  # FR
            position[0] -= self.leg_offset_x
            position[2] += self.leg_offset_y
        
        elif id == 2:  # RL
            position[0] += self.leg_offset_x
            position[2] -= self.leg_offset_y
        
        elif id == 3:  # RR
            position[0] -= self.leg_offset_x
            position[2] -= self.leg_offset_y

        return position
    
    def apply_rpy(self,x,y,z,roll,pitch,yaw):
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
        
        # rot_1 = np.matmul(rotate_z,rotate_x)
        # rot_2 = np.matmul(rot_1, rotate_y)
        # vector = np.matmul(rot_2, np.matrix([[x],[y],[z]]))
        
        vector = rotate_x @ rotate_y  @ rotate_z @ np.matrix([[x],[y],[z]])
        
        x_new = vector.item(0)
        y_new = vector.item(1)
        z_new = vector.item(2)
        return [x_new, y_new, z_new]
