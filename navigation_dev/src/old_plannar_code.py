#!/usr/bin/env python

import rospy
import cv2
import time
import math
import apriltag
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 


ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
current_pose = [0,0,0]
current_destination = 1
threshold = 0.1

move = 0.0
stop = 1.0

WAYPOINTS = [[0,0,0],[1,0,0]]

def moveForward(distance, speed, direction):
    speed_l = -1*speed
    speed_r = -1*(speed+0.01)
    if direction < 0:
        speed_l = -1*speed_l
        speed_r = -1*speed_r
    for i in range(int(10*distance/59.7)):
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
    for i in range(int(c*angles/math.pi)):   # mapp steps (11 for pi?) to required angle
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    time.sleep(1.0)

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




def moveRobot(x1,y1,initialAngle, x2, y2, finalAngle):
    angleForTravel = getAngleForTravel(x1,x2,y1,y2)
    print(angleForTravel, "angle for travel")
    angleToBeRotated = getAngleToRotate(initialAngle, angleForTravel)
    print(angleToBeRotated, " angle to be rotated")
    if angleToBeRotated >=1.5*math.pi or angleToBeRotated <= 0.5*math.pi:
        if angleToBeRotated >= 1.5*math.pi:
            angleToBeRotated -= 2*math.pi
            orientation = "clockwise"
        else:
            orientation = "anticlockwise"
        direction = 1
    else:
        if angleToBeRotated >= math.pi:
            angleToBeRotated -= math.pi
            orientation = "anticlockwise"
            direction = -1
        else:
            angleToBeRotated -= math.pi
            orientation = "clockwise"
            direction = -1
    print(angleToBeRotated, orientation, direction)
    rotate(abs(angleToBeRotated),0.7,orientation)

    if direction < 0 :
        angleForTravel = (angleForTravel + math.pi)%(2*math.pi)
    distance = getDistance(x1,y1,x2,y2)
    moveForward(distance*100,0.7,direction)
    finalRotation = getAngleToRotate(angleForTravel, finalAngle)
    if finalRotation > 2*math.pi - 0.09:
        finalRotation = 0.00
    print(finalRotation, "finalRotation")
    if angleToBeRotated >math.pi:
        orientation = "clockwise"
        angleToBeRotated = 2*math.pi - angleToBeRotated
    else:
        orientation = "anticlockwise"
    rotate(abs(finalRotation), 0.7, orientation)
    print()


def pose_callback(msg):
    print("current destination :: ")
    global current_destination
    
    print(current_destination)
    if current_destination > 1:
        return

    # TODO: estimate control actions 
    cmd_msg = Float32MultiArray()
    print("updating current pose")

    matrix = msg.pose.matrix
    if matrix == None:
        return
    y = matrix[0]
    x = matrix[1]
    theta = matrix[2]
    
    global current_pose
    current_pose[0] = x
    current_pose[1] = y
    current_pose[2] = theta

    dest_x = WAYPOINTS[current_destination][0]
    dest_y = WAYPOINTS[current_destination][1]
    finalAngle = WAYPOINTS[WAYPOINTS[current_destination][2]]

    if(getDistance(x,y,dest_x,dest_y) < threshold):
        current_destination = current_destination + 1
        dest_x = WAYPOINTS[current_destination][0]
        dest_y = WAYPOINTS[current_destination][1]
        print("moving to new wavepoint")

    print(current_pose)
    print(current_destination)

    if current_destination > 1:
        print("Final Wavepoint reached")
        return

    # moveRobot(x,y,initialAngle,dest_x,dest_y,finalAngle)



if __name__ == "__main__":
    print("I am here")
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", Pose, pose_callback)
    # if(current_destination <= 1):
    #     rospy.spin()

    x = current_pose[0]
    y = current_pose[1]
    initialAngle = current_pose[2]

    dest_x = WAYPOINTS[current_destination][0]
    dest_y = WAYPOINTS[current_destination][1]
    finalAngle = WAYPOINTS[WAYPOINTS[current_destination][2]]

    while current_destination <=1 and abs(getDistance(x,y,dest_x,dest_y)) > 0.05:
        print(current_pose)
        print(current_destination)
        dest_x = WAYPOINTS[current_destination][0]
        dest_y = WAYPOINTS[current_destination][1]
        finalAngle = WAYPOINTS[WAYPOINTS[current_destination][2]]

        x = current_pose[0]
        y = current_pose[1]
        initialAngle = current_pose[2]
        
        moveRobot(x,y,initialAngle,dest_x,dest_y,finalAngle)
        count = count + 1        
        if count > 10: 
            break