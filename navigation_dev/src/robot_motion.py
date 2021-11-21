#!/usr/bin/env python

import rospy
import time
import math
import apriltag
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Robot_Pose 
from navigation_dev.msg import Robot_motion_control
from EKF import jetbotEKF
import numpy as np

ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
current_pose = [0,0,0]
measurement = [0,0,0]
world_land = [0,0]
t = 0

ekf = jetbotEKF()

def call(l, z, id, final_pose, vt, t):

    if id not in ekf.landmarks:
        ekf.add_landmark(l,id)
    ind = np.where(ekf.landmarks == id)[0]
    zm = np.zeros(ekf.ms)
    zm[ind*2] = z[0]
    zm[ind*2+1] = z[1]
    ekf.predict(final_pose, vt, t)
    ekf.update(np.array(zm))

    print(ekf.x)
    print()



def moveForward(distance, speed, direction):
    speed_l = -1*speed
    speed_r = -1*(speed+0.015)
    if direction < 0:
        speed_l = -1*speed_l
        speed_r = -1*speed_r
    # for i in range(int(20*distance/59.7)):
    for i in range(int(20*distance/59.7)):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    # time.sleep(1.0)

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
    #### feedback correction
    # c = 8
    for i in range(int(c*angles/math.pi)):   # mapp steps (11 for pi?) to required angle
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    time.sleep(1.0)

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

def getDistance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def moveRobot(x1,y1,theta, x2, y2):
    global current_pose, t
    angleForTravel = getAngleForTravel(x1,x2,y1,y2)
    print(angleForTravel, "angle for travel")
    angleToBeRotated = (angleForTravel - theta)%(2*math.pi)
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
    # if abs(angleToBeRotated) > 0.5:
    rotate(abs(angleToBeRotated),0.7,orientation)
    print("ROTATED by ", angleToBeRotated, orientation)
        # if abs(angleToBeRotated) > 0.8:
        #     return

    distance = getDistance(x1,y1,x2,y2)
    t = time.start()
    moveForward(distance*100,0.7,direction)
    t = time.end() -t

    current_pose = [x2, y2, angleForTravel]

def pose_callback(msg):
    global measurement, world_land
    # x = msg.x
    # y = msg.y
    # theta = msg.theta
    # current_pose = [x,y,theta]
    zx = msg.x
    zy = msg.y
    id = msg.id

    measurement = [zx, zy, id]
    
    xr = ekf.x[0]
    yr = ekf.x[1]
    theta = ekf.x[2]

    x = zx*np.cos(theta) - zy*np.sin(theta) + xr
    y = zx*np.sin(theta) + zy*np.cos(theta) + yr

    world_land = [x,y]

if __name__ == "__main__":
    rospy.init_node('robot_motion')
    rospy.Subscriber("/current_pose", Robot_motion_control, pose_callback)

    move = 0.0
    stop = 1.0
    vt = 0.32
    w = np.pi/4

    R = 0.5

    WAYPOINTS = [[0,0,0],[R*np.cos(np.pi/4), R - R*np.sin(np.pi/4),0]]#,[R,R,0],[0,2,0]]#,[-1,1,1.57],[-2,1,0],[-2,2,-1.57],[-1,1,-0.78],[0,0,0]]
    # WAYPOINTS = [[-1,1,0],[-2,1,0],[-2,2,-1.57]]
    for i in range(1,len(WAYPOINTS)):
        
        call(world_land, measurement[0:2], measurement[2], current_pose, vt, t, w)

        # while not can_robot_move:
        #     continue
        moveRobot(current_pose[0],current_pose[1],current_pose[2],WAYPOINTS[i][0],WAYPOINTS[i][1])
        

