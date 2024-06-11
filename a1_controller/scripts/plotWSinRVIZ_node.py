#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from A1_kinematics import get_pw, calc_joint_angles

leg_offset_y = 0.1805
leg_offset_x = 0.047

def publish_pointcloud(posAndSolList, name):
    rospy.init_node('pointcloud_publisher', anonymous=True)
    pointcloud_pub = rospy.Publisher(f'/pointcloud{name}', PointCloud, queue_size=10)

    try:
        pointcloud_msg = PointCloud()
        pointcloud_msg.header.stamp = rospy.Time.now()
        pointcloud_msg.header.frame_id = 'base' # 'FL_hip', use base instead of hip because hip movement and pointcloud orientation
                                                #  should be independent

        for position in posAndSolList:
            # Word coordinates and DH coordinates are transformed
            rot_dh_rviz = np.array([
                [0,-1,0],
                [0,0,-1],
                [-1,0,0]
            ])
                
            position_rviz = position @ rot_dh_rviz
            
            position_rviz[0] += leg_offset_y  # shift from base to FL_hip
            position_rviz[1] += leg_offset_x
            
            point = Point32(*[p for p in position_rviz])

            pointcloud_msg.points.append(point)
            
        pointcloud_pub.publish(pointcloud_msg)
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    num_samples = 300000
    theta0_range = np.random.uniform(-0.8, 0.8, num_samples)
    theta2_range = np.random.uniform(-1.05 -np.pi/2, 4.18 - np.pi/2, num_samples)
    theta3_range = np.random.uniform(-2.69, -0.92, num_samples)
    

    end_effector_positions = []

    for i in range(num_samples):
        theta0 = theta0_range[i]
        theta2 = theta2_range[i]
        theta3 = theta3_range[i]
        
        x_end, y_end, z_end = get_pw(theta0, theta2, theta3)
        end_effector_positions.append([x_end, y_end, z_end])

    end_effector_positions = np.array(end_effector_positions)


    #find out all possible solution to each end_effector_position
    unique_end_effector_position = []
    redundant_end_effector_position = []
    
    for end_effector_position in end_effector_positions:
        solutions =calc_joint_angles(end_effector_position, True)
        if len(solutions) == 1:
            #print("----------unique solution found-----")
            unique_end_effector_position.append(end_effector_position)
        elif len(solutions) > 1:
            #print("-----------REDUNDANT solutions-------")
            redundant_end_effector_position.append(end_effector_position)
    
    try:
     while not rospy.is_shutdown():
        publish_pointcloud(unique_end_effector_position, "_unique")
        publish_pointcloud(redundant_end_effector_position, "_redundant")

    except rospy.ROSInterruptException:
        pass
