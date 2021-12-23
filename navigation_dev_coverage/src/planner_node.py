#!/usr/bin/env python

import rospy
import cv2
import time
import math
import apriltag
import numpy as np
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 


ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
current_pose = [0,0,0]
estimated_current_pose = [0,0,0]
current_destination = 1
threshold = 0.1

move = 0.0
stop = 1.0

WAYPOINTS = [[0.1, -0.0, 0.0], [0.3, -0.0, 0.0], [0.5, -0.0, 0.0], [0.7, -0.0, 0.0], [0.9, -0.0, 0.0], [1.1, -0.0, 0.0], 
[1.1, -0.0, 0.0], [1.1, -0.1, 0.0], [0.9, -0.1, 0.0], [0.7, -0.1, 0.0], [0.5, -0.1, 0.0], [0.3, -0.1, 0.0], [0.1, -0.1, 0.0], 
[0.1, -0.2, 0.0], [0.3, -0.2, 0.0], [0.5, -0.2, 0.0], [0.7, -0.2, 0.0], [0.9, -0.2, 0.0], [1.1, -0.2, 0.0], [1.1, -0.2, 0.0], 
[1.1, -0.3, 0.0], [0.9, -0.3, 0.0], [0.7, -0.3, 0.0], [0.5, -0.3, 0.0], [0.3, -0.3, 0.0], [0.1, -0.3, 0.0], [0.1, -0.4, 0.0], 
[0.3, -0.4, 0.0], [0.5, -0.4, 0.0], [0.7, -0.4, 0.0], [0.9, -0.4, 0.0], [1.1, -0.4, 0.0], [1.1, -0.4, 0.0], [1.1, -0.5, 0.0], 
[0.9, -0.5, 0.0], [0.7, -0.5, 0.0], [0.5, -0.5, 0.0], [0.3, -0.5, 0.0], [0.1, -0.5, 0.0]]

## as calculated by the algorithm


def moveForward(distance, speed, direction):
    speed_l = -1*speed
    speed_r = -1*(speed+0.01)
    if direction < 0:
        speed_l = -1*speed_l
        speed_r = -1*speed_r
    for i in range(min(5,int(10*distance/59.7))):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    time.sleep(1.0)

def rotate(angles, speed, direction):
    speed_l = -1*speed
    speed_r = -1*(speed)
    if direction == "clockwise":
        speed_r*=-1
    else:
        speed_l*=-1
    msg = Float32MultiArray()
    c = 10.5
    if angles > math.pi - math.pi/10:
        c = 8
    if angles < math.pi/2 - math.pi/10:
        c = 13
    for i in range(min(2,int(c*angles/math.pi))): 
    # for i in range(2):
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    time.sleep(0.5)

def getDistance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def getAngleToRotate(initialAngle, FinalAngle):
    FinalAngle = FinalAngle % (2*math.pi)
    initialAngle = initialAngle % (2*math.pi)
    return (FinalAngle - initialAngle)%(2*math.pi)

## Assuming we are getting angles from 0 to 360, NOPE, we're getting -pi/2 to pi/2 imo
def getAngleForTravel(x1,x2,y1,y2):
    eps = 10**(-10)
    num = y2-y1
    den = x2-x1
    atan = math.atan((y2-y1)/((x2-x1)+eps))
    if atan<0:
        if num>0 or den<0:
            atan = atan + math.pi
    else:
        if num<0 or den<0:
            atan+= math.pi
    return atan % (2*math.pi)



## This method is responsible for updating the estimated current pose of the robot
## At every iteration we either make the robot move by 0.15 units or the entire distance
## to its nect destination (which ever is smaller).
def moveRobot(x1,y1,initialAngle, x2, y2, finalAngle):
    angleForTravel = getAngleForTravel(x1,x2,y1,y2)
    # print(angleForTravel, "angle for travel")
    angleToBeRotated = getAngleToRotate(initialAngle, angleForTravel)
    # print(angleToBeRotated, " angle to be rotated")
    if angleToBeRotated <=math.pi:
        orientation = "anticlockwise"
        direction = 1
    else:
        angleToBeRotated -= 2*math.pi
        orientation = "clockwise"
        direction = 1
    if abs(angleToBeRotated) > 0.1:
        rotate(abs(angleToBeRotated), 0.7, orientation)
        if orientation is "clockwise":
            estimated_angle = estimated_current_pose[0] + angleToBeRotated
        else:
            estimated_angle = estimated_current_pose[0] - angleToBeRotated
        estimated_angle[2] = estimated_angle

    if direction < 0 :
        angleForTravel = (angleForTravel + math.pi)%(2*math.pi)
    distance = getDistance(x1,y1,x2,y2)
    if abs(angleToBeRotated) < 0.8:
        moveForward(distance*100,0.7,direction)
        estimated_angle = estimated_current_pose[2]
        estimated_x = min(0.15,distance) * np.cos(estimated_angle)
        estimated_y = min(0.15,distance) * np.sin(estimated_angle)
        estimated_angle[0] = estimated_x
        estimated_angle[1] = estimated_y
    print()

## This method compares the estimated position of the robot with the global position 
## calculated by each landmark. The landmark that gives the least difference is chosen 
## and the global position of the robot is updated accordingly
def getMostProbablePosition(positions):

    index = 0
    error = 1000000
    for i in range(len(positions)):
        p = positions[i]
        dist = math.pow((p[0] - estimated_current_pose[0]),2) + math.pow((p[1] - estimated_current_pose[1]),2)
        if dist<error:
            error = dist
            index = i
    return positions[index]


def pose_callback(msg):
    # print("current destination :: ")
    global current_destination
    cmd_msg = Float32MultiArray()

    positions = msg.pose.matrix
    
    matrix = getMostProbablePosition(positions)

    if matrix == None:
        return
    x = matrix[0]
    y = matrix[1]
    theta = matrix[2]
    

    global current_pose
    current_pose[0] = x
    current_pose[1] = y
    current_pose[2] = theta

    dest_x = WAYPOINTS[current_destination][0]
    dest_y = WAYPOINTS[current_destination][1]
    finalAngle = WAYPOINTS[WAYPOINTS[current_destination][2]]

    if getDistance(x,y,dest_x,dest_y) < threshold and current_destination +1 < len(WAYPOINTS):
        current_destination = current_destination + 1
        dest_x = WAYPOINTS[current_destination][0]
        dest_y = WAYPOINTS[current_destination][1]
        print("moving to new wavepoint")


if __name__ == "__main__":
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    # if(current_destination <= 1):
    #     rospy.spin()

    x = current_pose[0]
    y = current_pose[1]
    initialAngle = current_pose[2]

    dest_x = WAYPOINTS[current_destination][0]
    dest_y = WAYPOINTS[current_destination][1]
    finalAngle = WAYPOINTS[current_destination][2]
    count = 0
    while current_destination < len(WAYPOINTS) and abs(getDistance(x,y,dest_x,dest_y)) > 0.03:
        print(current_pose)
        print(WAYPOINTS[current_destination])
        dest_x = WAYPOINTS[current_destination][0]
        dest_y = WAYPOINTS[current_destination][1]
        finalAngle = WAYPOINTS[current_destination][2]

        x = current_pose[0]
        y = current_pose[1]
        initialAngle = current_pose[2]
        
        moveRobot(x,y,initialAngle,dest_x,dest_y,finalAngle)
        count = count + 1        
        if count > 10000: 
            break
    
    print("Fin")