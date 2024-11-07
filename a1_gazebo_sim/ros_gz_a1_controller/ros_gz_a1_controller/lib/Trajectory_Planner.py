import numpy as np

class Trajectory_Planner:  
    """Class used to bundle trajectory generation and other helpful
                functions related to the foot positions""" 
                 
    def __init__(self, base_height=0.225) -> None:
        self.leg_offset_y = 0.1805
        self.leg_offset_x = 0.047
        self.base_height = base_height
        self.z_fd = 0.0

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
    


    import numpy as np

    def trot(self, legIdx, position, step_height, T_period, t, linear_vel=[0,0], command_vel=[0,0], k_z=0.024, k_y=0.0001):
        x = position[0]   
        y = position[1]   
        z = position[2]   
        desired_vel_z = command_vel[0]  
        linear_vel_z = linear_vel[0]    

        T_swing = T_period / 2
        T_stand = T_period / 2

        if legIdx in (0, 3):  # For legs 0 and 3, adjust time offset for trotting
            t += T_period / 2
        t %= T_period

        # Raibert heuristic for forward foot placement (z-direction) 
        if t == 37:
            self.z_fd = ((linear_vel_z * 0.25) / 2 + k_z * (linear_vel_z - desired_vel_z))

        if t >= T_stand:  # Swing phase
            u = t - T_stand
            if np.abs(z) < 0.5:
                z -= self.z_fd/25
            else:
                z = np.sign(z) * 0.5 

            y = -step_height * np.sin(np.pi * u / T_swing) + self.base_height
        else:  # Stand phase

            y = self.base_height
            z += self.z_fd/25
        
        # Return the foot position for the leg
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
