#!/usr/bin/env python

import math
import rospy
import cv2
import apriltag
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
from navigation_dev.msg import TMatrix 
import numpy as np
from positions import *
from rotation import dummy, getAnglesFromRotationMatrix 

pose_pub = rospy.Publisher('/current_pose', AprilDetections, queue_size=2)
mapping_array = []



def tag_callback(msg):
    # TODO: implement localization logic 
    global mapping_array

    pose_msg = AprilDetections()
    pose_msg.header.stamp = msg.header.stamp
    # print(msg.ids[0])
    tag = msg.ids[0]
    map = {}
    map[1] = [6, 0, 2, 7, 11, 5,8]
    map[2] = [3, 1, 9, 10, 4]
    if tag in map[1]:
        tag = 1
    else:
        tag = 2
    # print(tag)


    # TODO
    detections = msg.detections
    # world_tags = returnPositions(tag)
    if len(detections)!=0:
        matrix = detections[0].matrix
        matrix = np.reshape(np.array(matrix), (4,4))
        (y,x) = (-1*matrix[0][3], matrix[2][3])
        # print(x,y)
        mapping_array.append([x,y,tag])
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
    #     pose_msg.pose.matrix = [x,y,tag]
    # else:
    #     pose_msg.pose.matrix = [0,0,-1]

    # print(matrix)

    # pose_msg.pose = [R11, R12, R13, t1,
    #                  R21, R22, R23, t2,
    #                  R31, R32, R33, t3]

    if len(mapping_array) >= 10:
        print(mapping_array)
        for i in range(len(mapping_array)):
            p = TMatrix()
            p.matrix += mapping_array[i]
            pose_msg.detections.append(p)
        pose_pub.publish(pose_msg) 
        mapping_array = [] 


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
