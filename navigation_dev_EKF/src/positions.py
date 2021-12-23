import numpy as np

import math

def returnCurrentPose(tag,x,z,world_tags):
    if tag == 1:
        return world_tags[0][0] + z, world_tags[0][1] + x

    if tag == 2:
        return  world_tags[0][0] + x, world_tags[0][1] + z

    if tag == 3:
        return world_tags[0][0] - x, world_tags[0][1] + z

    if tag == 4:
        return world_tags[0][0] + z, world_tags[0][1] + x

    if tag == 5:
        return world_tags[0][0] - x, world_tags[0][1] + z

    if tag == 6:
        return world_tags[0][0] + z, world_tags[0][1] + x

    if tag == 7:
        return world_tags[0][0] + x, world_tags[0][1] -z

    if tag == 8:
        return world_tags[0][0] - z, world_tags[0][1] - x

    if tag == 9:
        return world_tags[0][0] - z, world_tags[0][1] - x

    # return np.array([[0.00,2.032,0.00]])




def returnPositions(tag):
    if tag == 1:
        return np.array([[1.70,-0.04,0.24+0.1]])

    if tag == 2:
        return  np.array([[1.50,2.13,math.pi/2-0.2]])

    if tag == 3:
        return np.array([[1.54,2.02,math.pi/2+0.24]])

    # if tag == 4:
    #     return np.array([[0.00,0.00,math.pi]])

    if tag == 5:
        return np.array([[0.00,1.25,math.pi/2]])

    if tag == 4:
        return np.array([[2.032,0.53,0.0-0.20]])

    if tag == 7:
        return np.array([[0.7112,-0.6096,-0.5*math.pi]])

    if tag == 8:
        return np.array([[0.3,1.18,math.pi]])

    if tag == 9:
        return np.array([[0.0,0.0,math.pi]])

    # return np.array([[0.00,2.032,0.00]])