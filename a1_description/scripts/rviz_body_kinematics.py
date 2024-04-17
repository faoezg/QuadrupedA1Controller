#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import A1_kinematics

def joint_state_publisher():
  rospy.init_node('joint_state_publisher_node')

  joint_names = ["FR_hip_joint","FR_thigh_joint","FR_calf_joint","FL_hip_joint",
                      "FL_thigh_joint","FL_calf_joint","RR_hip_joint","RR_thigh_joint",
                      "RR_calf_joint","RL_hip_joint","RL_thigh_joint","RL_calf_joint"]
  joint_state_pub = rospy.Publisher('/hihi/joint_states', JointState, queue_size=10)
  joint_state_msg = JointState() # create joint state message
  joint_state_msg.header.stamp = rospy.Time.now() 
  joint_state_msg.name = joint_names                        
  joint_state_msg.velocity = []
  joint_state_msg.effort = []
  
  joint_state_pub.publish(joint_state_msg)
  rate = rospy.Rate(100)


  t = 0
  base_height = 20
  length_offset = 0
  base_width = 8.38
  while not rospy.is_shutdown():
    
    if 0<=t<50:
        length_offset -= 10/100
    elif 50<=t<150:
        length_offset += 10/100  
    elif 150<=t<200:
        length_offset -= 10/100
    
    if 200<=t<250:
        base_height -= 10/100
    elif 250<=t<350:
        base_height += 10/100  
    elif 350<=t<400:
        base_height -= 10/100
        
    if 400<=t<450:
        base_width -= 5/100
    elif 450<=t<550:
        base_width += 5/100  
    elif 550<=t<600:
        base_width -= 5/100
    

    joint_state_msg.header.stamp = rospy.Time.now() 
    
    """#calculate angles for certain base height -> Inverse Kinematics in Z direction
    h_c = np.sqrt(20**2 - (base_height/2)**2)
    th2 = np.arcsin(h_c/20) # in meinen notizen ist das curr
    th3 = -2*th2
    
    #calculate angles for certain base length offset -> Inverse Kinematics in Y direction
    d = np.sqrt(length_offset**2 + base_height**2)
    h_d = np.sqrt(4*(20**2) - d**2)/2
    beta = np.arcsin(h_d/20)
    x = np.arcsin(length_offset/base_height)"""
    print(length_offset)
    angles = A1_kinematics.calc_joint_angles([base_width, base_height, length_offset ])
    th0 = angles[0][0]
    th2 = angles[0][1]
    th3 = angles[0][2]
        
    goal_pos = [th0, th2 + np.pi/2, th3, th0, th2 + np.pi/2, th3,
                th0, th2 + np.pi/2, th3, th0, th2 + np.pi/2, th3]    
    joint_state_msg.position = goal_pos

    joint_state_pub.publish(joint_state_msg)
    print(A1_kinematics.get_pw(0,th2-np.pi/2, th3))
    print(t)
    t+=1
    t%=600
    rate.sleep()


if __name__ == '__main__':
  try:  
    joint_state_publisher()
  except rospy.ROSInterruptException:
    pass