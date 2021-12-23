#!/usr/bin/env python

import rospy
import time
import math
import apriltag
from std_msgs.msg import Float32MultiArray
from EKF import jetbotEKF
import numpy as np
from navigation_dev.msg import Pose
from navigation_dev.msg import AprilDetections


ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=2)
current_pose = [0,0,0]
measurement = [[0,0,-1]]
world_land = [[0,0]]
t = 0
cov = []
ekf = jetbotEKF()

move = 0.0
stop = 1.0


## method to call the predict and update method for Extended Kalman Filter Calculations
def call(l, z, id, final_pose, vt, t):
    print("current landmark seen:: ", id)
    # if id not in ekf.landmarks:
    #     ekf.add_landmark(l,id)
    addNew, ind = ekf.dataAssociation(l,id)  ## check data association
    
    # ind = np.where(np.array(ekf.landmarks) == id)[0]
    zm = np.zeros(ekf.ms)
    zm[ind*2] = z[0]
    zm[ind*2+1] = z[1]
    # print("state before update:: ", ekf.x)
    ekf.predict(final_pose, vt, t)
    ekf.update(np.array(zm), ind)
    # print("zm: ", zm)
    # print("state: ")
    print(ekf.x[0:3])
    for i in range(ekf.ms/2):
        print([round(ekf.x[2*i+3], 3), round(ekf.x[2*i+4],3)])
    print("landmark ids: ", ekf.landmarks)
    print("cov trace: ", np.trace(ekf.sigma))
    cov.append(round(np.trace(ekf.sigma),3))
    print("\n\n")


## method to move the robot forwards
def moveForward(distance, speed, direction):
    speed_l = -1*speed
    speed_r = -1*(speed+0.015)
    if direction < 0:
        speed_l = -1*speed_l
        speed_r = -1*speed_r
    # for i in range(int(20*distance/59.7)):
    for i in range(int(19*distance/59.7)):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    # time.sleep(1.0)

## method to rotate the robot
def rotate(angles, speed, direction):
    speed_l = -1*(speed)
    speed_r = -1*(speed)
    if direction == "clockwise":
        speed_r*=-1
        speed_l = -1*(speed+0.08)
    else:
        speed_l*=-1
    msg = Float32MultiArray()
    c = 10.5
    if angles > math.pi - math.pi/10:
        c = 8
    if angles < math.pi/2 - math.pi/10:
        c = 15
    #### feedback correction
    # c = 8
    for i in range(int(c*angles/math.pi)):   # mapp steps (11 for pi?) to required angle
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    time.sleep(1.0)


## method to get how much the robot needs to rotate to go to the next way point
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


## method to get distance for travel
def getDistance(x1,y1,x2,y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)


## method to move robot from one waypoint to next. After movement is completed, the call method is invoked
def moveRobot(x1,y1,theta, x2, y2):
    global current_pose, t, measurement, world_land
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
    current_pose = [x1, y1, angleForTravel]
    for p in range(len(measurement)):
        meas = measurement[p]
        world = world_land[p]
    # if measurement[2] != -1:
        print("measurement: ", meas)
        call(world, meas[0:2], meas[2], current_pose, vt=0, t=0)
    print("\n**********************\n\n")
    # if measurement[2] != -1:
    #     call(world_land, measurement[0:2], measurement[2], current_pose, vt=0, t=0) # no translation
    #     time.sleep(0.3)
    #     call(world_land, measurement[0:2], measurement[2], current_pose, vt=0, t=0) # no translation
    #     time.sleep(0.3)
    #     call(world_land, measurement[0:2], measurement[2], current_pose, vt=0, t=0) # no translation
        
    distance = getDistance(x1,y1,x2,y2)
    t = time.time()
    moveForward(distance*100,0.7,direction)
    t = time.time() -t

    current_pose = [x2, y2, angleForTravel]
    print(current_pose)

## This method continuously receives measurement matrices from the localization node
## It receives a total of ten reading from its surrounding April tags
def pose_callback(msg):
    global measurement, world_land
    # x = msg.x
    # y = msg.y
    # theta = msg.theta
    # current_pose = [x,y,theta]
    msg = msg.detections
    # zx = msg[0]
    # zy = msg[1]
    # id = msg[2]
    measurement = []

    for i in range(len(msg)):
        measurement.append(msg[i].matrix)
    
    xr = ekf.x[0]
    yr = ekf.x[1]
    theta = ekf.x[2]
    world_land = []

    for p in measurement:
        zx = p[0]
        zy = p[1]
        id = p[2]

        x = zx*np.cos(theta) - zy*np.sin(theta) + xr
        y = zx*np.sin(theta) + zy*np.cos(theta) + yr

        world_land.append([x,y])



if __name__ == "__main__":
    rospy.init_node('planner_node')
    rospy.Subscriber("/current_pose", AprilDetections, pose_callback)

    move = 0.0
    stop = 1.0
    vt = 0.32
    w = np.pi/3

    R = 0.5

    # rotate(np.pi/4, 0.7, "clockwise")
    
    
    # rotate(np.pi/4, 0.7, "anticlockwise")

    # WAYPOINTS = [[0,0,0],[R*np.cos(np.pi/4), R - R*np.sin(np.pi/4),0],[R,R,0],
    #  [R*np.cos(np.pi/4),R + R*np.sin(np.pi/4),0], [0, 2*R,0],
    #  [-R*np.cos(np.pi/4),R + R*np.sin(np.pi/4),0], [-R,R,0],
    #  [-R*np.cos(np.pi/4), R - R*np.sin(np.pi/4),0], [0,0,0]]
    WAYPOINTS = []
    ang = np.pi/6

    #circle
    for i in range(1,13):
        a = -1*np.pi/2 + (i)*ang
        x = R*np.cos(a)
        y = R + R*np.sin(a)
        WAYPOINTS.append([x,y,0])

    # #eight
    # for i in range(1,13):
    #     a = -1*np.pi/2 + (i)*ang
    #     x = R*np.cos(a)
    #     y = -R - R*np.sin(a)
    #     WAYPOINTS.append([x,y,0])

    # print(WAYPOINTS)
    # WAYPOINTS[0][2] = -math 


    ## We made our robot run in one circle, then in two circles, then in four circles
    ## and then in the form of figure eight

    for j in range(0,2):
        for i in range(1,len(WAYPOINTS)):
            moveRobot(current_pose[0],current_pose[1],current_pose[2],WAYPOINTS[i][0],WAYPOINTS[i][1])
            for p in range(len(measurement)):
                meas = measurement[p]
                world = world_land[p]
            # if measurement[2] != -1:
                print("measurement: ", meas)
                if p!=0:
                    call(world, meas[0:2], meas[2], current_pose, vt=0, t=0)
                else:
                    call(world, meas[0:2], meas[2], current_pose, vt, t)
            print("\n------------------------------\n\n")
            time.sleep(2.0)
            print("state: ", [WAYPOINTS[i][0],WAYPOINTS[i][1]] )
            print(ekf.x[0:2])
        print(cov)