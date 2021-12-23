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


## We are dealing with only two landmarks. This method gets a list of all possible
## global positions of the robot based on all the positions of the current landmark detected.
## These values are used by the plannar node.
def tag_callback(msg):
    # TODO: implement localization logic 
    global mapping_array

    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    # print(msg.ids[0])
    # tag_precidence = [1,8,2,4,7,9,0,6]
    bool = True
    # for tag in tag_precidence:
    #     if tag in msg.ids:
    #         idx = msg.ids.index(tag)
    #         bool = True
    #         break

    # TODO
    detections = msg.detections
    # world_tags = returnPositions(tag)
    if bool:
        matrix = detections[idx].matrix
        matrix = np.reshape(np.array(matrix), (4,4))
        # if tag == 11:
        print("tag, x, y :: ",tag,matrix[2][3],matrix[0][3])
        matrix = np.linalg.inv(matrix)
        possible_coordinates = returnGlobalCoordinates1(tag, matrix)
        matrix = []
        for i in range(len(possible_coordinates)):
            for coordinates in possible_coordinates:
                (y,x) = (-1*coordinates[0][3], coordinates[2][3])
                R = coordinates[0:3, 0:3]
                rotations = dummy(R)
                theta = rotations[1]%(2*math.pi)
                theta = (-1*rotations[1]+math.pi)%(2*math.pi)
                possible_current_pose = [x,y,theta]
                print(possible_current_pose)
                matrix.append(possible_current_pose)
            # pose_msg.pose.matrix.append(possible_current_pose)
        pose_msg.pose.matrix = matrix
        pose_pub.publish(pose_msg)
        print("-----------------------------")


if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback)
    rospy.spin()
