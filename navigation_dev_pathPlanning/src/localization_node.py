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
from rotation import dummy 

pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)
mapping_array = []



def tag_callback(msg):
    # TODO: implement localization logic 
    global mapping_array

    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    # print(msg.ids[0])
    tag = msg.ids[0]
    # map = {}
    # map[1] = [6, 0, 2, 7, 11, 5,8]
    # map[2] = [3, 1, 9, 10, 4]
    # if tag in map[1]:
    #     tag = 1
    # else:
    #     tag = 2
    # print(tag)


    # TODO
    detections = msg.detections
    world_tags = returnPositions(tag)
    if len(detections)!=0:
        matrix = detections[0].matrix
        matrix = np.reshape(np.array(matrix), (4,4))
        # if tag == 11:
        print("tag, x, y :: ",tag,matrix[2][3],matrix[0][3])
        matrix = np.linalg.inv(matrix)
        # print(matrix[2][3], -1*matrix[0][3])
        matrix = returnGlobalCoordinates(tag, matrix)
        (y,x) = (-1*matrix[0][3], matrix[2][3])
        # print(x,y)
        # mapping_array.append([x,y,tag])
        R = matrix[0:3, 0:3]
        rotations = dummy(R)
        theta = rotations[1]%(2*math.pi)
        # if tag == 2:
        #     print("theta :: ", theta)
        # print(x,y,theta)
        current_pose = [x,y,theta]
        # if current_pose[2] >= 6.2 or current_pose[2]<0.1:
        #     current_pose[2] = 0
        # if tag==11:
        print(current_pose)
        print("-----")
        pose_msg.pose.matrix = current_pose
        pose_pub.publish(pose_msg)



if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
