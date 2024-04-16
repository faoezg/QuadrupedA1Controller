#!/usr/bin/env python3
import numpy as np

# our constant a0,a1,a2 DH Parameters 
a0,a2,a3 = [8.38, 20.0, 20.0]

# constants
th0_max = 0.80
th0_min = -0.80 

th2_max = 4.18  - np.pi/2
th2_min = -1.05 - np.pi/2

th3_max = -0.92
th3_min = -2.69

def get_pw(th0, th2, th3):
    px = a0 * np.cos(th0) + np.sin(th0) * (a2 * np.sin(th2) + a3 * np.sin(th2 + th3))
    py = a0 * np.sin(th0) - np.cos(th0) * (a2 * np.sin(th2) + a3 * np.sin(th2 + th3))
    pz = a2 * np.cos(th2) + a3 * np.cos(th2 + th3)
    return [px, py, pz]


class InverseKinematics():
    def __init__(self, a0, a2, a3):
        self.a0 = a0
        self.a2 = a2
        self.a3 = a3

    def calc_theta3(self, px, py, pz):
        z = (px ** 2 + py ** 2 + pz ** 2 - self.a0 ** 2 - self.a2 ** 2 - self.a3 ** 2)
        n = (2 * self.a2 * self.a3)
        cost3 = z/n
        sp = np.sqrt(1 - cost3 ** 2)

        th3 = np.arctan2(sp, cost3)
        return [th3, -th3]

    def calc_theta2(self, th3, px, py, pz):
        s21z = np.sqrt(px ** 2 + py ** 2 - self.a0 ** 2) * (self.a2 + self.a3 * np.cos(th3)) - self.a3 * np.sin(th3) * pz  # pos sqrt
        s22z = -np.sqrt(px ** 2 + py ** 2 - self.a0 ** 2) * (self.a2 + self.a3 * np.cos(th3)) - self.a3 * np.sin(th3) * pz # neg sqrt
        
        c21z = np.sqrt(px ** 2 + py ** 2 - self.a0 ** 2) * self.a2 * np.sin(th3) + (self.a2 + self.a3 * np.cos(th3)) * pz
        c22z = -np.sqrt(px ** 2 + py ** 2 - self.a0 ** 2) * self.a2 * np.sin(th3) + (self.a2 + self.a3 * np.cos(th3)) * pz

        th21 = np.arctan2(s21z, c21z)
        th22 = np.arctan2(s22z, c22z)
        
        return th21, th22
    
    def calc_theta0(self, px, py):
        c0 = (a0*px - py*(np.sqrt(px**2 + py**2 - a0**2))) # case positive sqrt
        c0_1 = (a0*px + py*(np.sqrt(px**2 + py**2 - a0**2)))  # case negative sqrt

        s0 = (a0*py + px*(np.sqrt(px**2 + py**2 - a0**2))) # case pos sqrt
        s0_1 = (a0*py - px*(np.sqrt(px**2 + py**2 - a0**2)))  # case neg sqrt

    
        th0_1 = np.arctan2(s0, c0)
        th0_2 = np.arctan2(s0_1, c0_1)

        return [th0_1, th0_2]

    
def calc_joint_angles(position):
    ik = InverseKinematics(a0,a2,a3)
    
    # calculate all theta3s (calf):
    th3s = th3_0, th3_1 = ik.calc_theta3(*position)
    
    # calculate all theta2s, depending on theta3s       
    th2_0, th2_1 = ik.calc_theta2(th3_0, position[0],position[1],position[2]) # positive sin3 theta3s
    th2_2, th2_3 = ik.calc_theta2(th3_1, position[0],position[1],position[2]) # negative sin3 theta3s

    th2s = [th2_0, th2_1, th2_2, th2_3]
    
    # calculate all th0s depending on x and y coordinate
    th0s = ik.calc_theta0(position[0], position[1])
            
    sol1 = [th0s[0], th2s[0], th3s[0]]    # sol1/sol2 dont fit our joint limitations, since theta3 is positive here
    sol2 = [th0s[1], th2s[1], th3s[0]]
    
    sol3 = [th0s[0], th2s[2], th3s[1]]
    sol4 = [th0s[1], th2s[3], th3s[1]]    
        
    
    solutions = [sol1, sol2, sol3,sol4]
    fitting_ths = [] 
    for ts in solutions:  # check whether our ths fit within our joint constraints
        if th0_min <= ts[0] <= th0_max and th2_min <= ts[1] <= th2_max and th3_min <= ts[2] <= th3_max:        
            fitting_ths.append(ts)
    print(f"Corrent solutions: {len(fitting_ths)}, Th0s: {th0s}, y-pos: {position[1]}")
    return fitting_ths