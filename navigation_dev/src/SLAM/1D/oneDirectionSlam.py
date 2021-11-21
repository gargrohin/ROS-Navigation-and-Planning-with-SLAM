
from typing import Mapping
import rospy
import cv2
import time
import math
import apriltag
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
from navigation_dev.msg import Mapping1 

from collections import namedtuple
gaussian = namedtuple('Gaussian', ['mean', 'var'])
gaussian.__repr__ = lambda s: '𝒩(μ={:.3f}, 𝜎²={:.3f})'.format(s[0], s[1])
from oneDirectionKalman import *

ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
tag_pub = rospy.Publisher('/one_direction_map', Mapping1, queue_size=2)
current_pose = 0
current_ideal_pose = 0
current_kestimated_pose = 0
process_var = 1. # variance in the robot's movement
sensor_var = 2. # variance in the camera

def moveForward(distance, speed, direction):
    speed_l = -1*speed
    speed_r = -1*(speed+0.015)
    if direction < 0:
        speed_l = -1*speed_l
        speed_r = -1*speed_r
    # for i in range(int(20*distance/59.7)):
    for i in range(min(10,int(20*distance/59.7))):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    # time.sleep(1.0)

def pose_callback(msg):
    # TODO: estimate control actions 
    cmd_msg = Float32MultiArray()

    x = msg.x_position
    if x == None:
        return
    
    global current_pose
    current_pose = x

    pose_msg = Mapping1()
    pose_msg.id = msg.header.stamp
    pose_msg.x = msg.april_tag_x + current_kestimated_pose

    tag_pub.publish(pose_msg) 

    ## Assuming pose_msg.x to be the ideal position of the april tag

    ideal_Robot_Position = pose_msg.x + msg.robot_x
    kalmanEstimatedPositions(ideal_Robot_Position)


def moveForwardStep(old_position):
    for i in range(0,10):
        moveForward(1,0.7,1)
        tmp = current_ideal_pose
        current_ideal_pose = old_position + 0.7
        old_position = tmp
        kalmanEstimatedPositions(current_ideal_pose)
        time.sleep(2.0)


def moveBackwardStep(old_position):
    for i in range(0,10):
        moveForward(1,0.7,-1)
        tmp = current_ideal_pose
        current_ideal_pose = old_position - 0.7
        old_position = tmp
        kalmanEstimatedPositions(current_ideal_pose)
        time.sleep(2.0)

def kalmanEstimatedPositions(current_ideal_position):
    process_model = gaussian(0.7*1, process_var) 
    global current_kestimated_pose
    prior = predict(current_kestimated_pose,process_model)
    likelihood = gaussian(current_ideal_position,sensor_var)
    current_kestimated_pose = update(prior, likelihood)

if __name__ == "__main__":
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)

    move = 0.0
    stop = 1.0

    for i in range(1,20):
        moveForwardStep()
        moveBackwardStep()