#!/usr/bin/env python

import math
import rospy
import cv2
import apriltag
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np
from positions import *
from rotation import dummy, getAnglesFromRotationMatrix
from EKF import jetbotEKF 

pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)

def tag_callback(msg):
    # TODO: implement localization logic

    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    print(msg.ids[0])
    tag = msg.ids[0]




    # TODO
    detections = msg.detections
    world_tags = returnPositions(tag)
    if len(detections)!=0:
        matrix = detections[0].matrix
        matrix = np.reshape(np.array(matrix), (4,4))
        (zy,zx) = (-1*matrix[0][3], matrix[2][3])

        xr = ekf.x[0]
        yr = ekf.x[1]
        theta = ekf.x[2]

        x = zx*np.cos(theta) - zy*np.sin(theta) + xr
        y = zx*np.sin(theta) + zy*np.cos(theta) + yr




        # R = matrix[0:3, 0:3]
        # rotations = dummy(R)
        
        ## According to rotation matrix, z will be x, and x will be y
        # print()
        # print(x)
        # print(z)
        # # print(rotations[1])
        # print()
        # cp_x, cp_y = returnCurrentPose(tag,x,z,world_tags)
        # current_pose = [cp_x, cp_y,((world_tags[0][2])%(2*math.pi) + rotations[1]%(2*math.pi))%(2*math.pi)] 
        # if current_pose[2] >= 6.2:
        #     current_pose[2] = 0
        # print(current_pose)
        # print(x,z)

        # pose_msg.pose.matrix = current_pose
        pose_msg.pose.matrix = [x]
    # print(matrix)

    # pose_msg.pose = [R11, R12, R13, t1,
    #                  R21, R22, R23, t2,
    #                  R31, R32, R33, t3]

    pose_pub.publish(pose_msg)  


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
