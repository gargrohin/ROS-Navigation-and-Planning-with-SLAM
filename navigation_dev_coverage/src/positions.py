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
        return  np.array([[1.0,0.0,0.0]])

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

## HOMEWORK 5 ##
def returnGlobalCoordinates1(tag, matrix):
    global_coordinates1 = []
    global_coordinates2 = []

    Tag_mat1 = np.array([[1.0,0.0,0.0,0.53],[0,1,0,0],[0.0,0,1.0,1.70],[0,0,0,1]])
    global_coordinates1.append(np.matmul(Tag_mat1, matrix))

    Tag_mat2 = np.array([[0,0,1,0.99],[0,1,0,0],[-1,0,0,1.62],[0,0,0,1]])
    global_coordinates2.append(np.matmul(Tag_mat2, matrix))

    Tag_mat8 = np.array([[1,0,0,0.-0.08],[0,1,0,0],[0,0,1,1.72],[0,0,0,1]])
    global_coordinates2.append(np.matmul(Tag_mat8, matrix))

    Tag_mat6 = np.array([[0,0,1,0.83],[0,1,0,0],[-1,0,0,0.90],[0,0,0,1]])
    global_coordinates2.append(np.matmul(Tag_mat6, matrix))

    Tag_mat7 = np.array([[0,0,1,1.18],[0,1,0,0],[-1,0,0,0.33],[0,0,0,1]])
    global_coordinates1.append(np.matmul(Tag_mat7, matrix))

    Tag_mat5 = np.array([[1,0,0,0.-0.90],[0,1,0,0],[0,0,1,2.05],[0,0,0,1]])
    global_coordinates1.append(np.matmul(Tag_mat5, matrix))

    Tag_mat0 = np.array([[-1,0,0,-0.09],[0,1,0,0],[0,0,-1,-0.20],[0,0,0,1]])
    global_coordinates2.append(np.matmul(Tag_mat0, matrix))

    Tag_mat4 = np.array([[0,0,-1,0.-0.74],[0,1,0,0],[1,0,0,1.44],[0,0,0,1]])
    global_coordinates2.append(np.matmul(Tag_mat4, matrix))

    Tag_mat11 = np.array([[-1.0,0,0,0.-0.85],[0,1,0,0],[0,0,-1.0,-0.10],[0,0,0,1]])
    global_coordinates1.append(np.matmul(Tag_mat11, matrix))

    Tag_mat9 = np.array([[-1,0,0,0.65],[0,1,0,0],[0,0,-1,-0.34],[0,0,0,1]])
    global_coordinates1.append(np.matmul(Tag_mat9, matrix))

    Tag_mat3 = np.array([[1,0,0,0.60],[0,1,0,0],[0,0,1,1.69],[0,0,0,1]])
    global_coordinates1.append(np.matmul(Tag_mat3, matrix))

    if tag%2 == 0:    ### change
        return global_coordinates2
    return global_coordinates1

        

def returnGlobalCoordinates(tag, matrix):
    if tag == 3:
        Tag_mat = np.array([[1,0,0,0.60],[0,1,0,0],[0,0,1,1.69],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)
    if tag == 2:
        Tag_mat = np.array([[0,0,1,0.99],[0,1,0,0],[-1,0,0,1.62],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)
    if tag == 8:
        Tag_mat = np.array([[1,0,0,0.-0.08],[0,1,0,0],[0,0,1,1.72],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)
    if tag == 6:
        Tag_mat = np.array([[0,0,1,0.83],[0,1,0,0],[-1,0,0,0.90],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)
    if tag == 7:
        Tag_mat = np.array([[0,0,1,1.18],[0,1,0,0],[-1,0,0,0.33],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)
    if tag == 5:
        Tag_mat = np.array([[1,0,0,0.-0.90],[0,1,0,0],[0,0,1,2.05],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)

    if tag == 0:
        Tag_mat = np.array([[-1,0,0,-0.09],[0,1,0,0],[0,0,-1,-0.20],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)

    if tag == 1:
        Tag_mat = np.array([[1.0,0.0,0.0,0.53],[0,1,0,0],[0.0,0,1.0,1.70],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)

    if tag == 4:
        Tag_mat = np.array([[0,0,-1,0.-0.74],[0,1,0,0],[1,0,0,1.44],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)

    if tag == 11:
        Tag_mat = np.array([[-1.0,0,0,0.-0.85],[0,1,0,0],[0,0,-1.0,-0.10],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)

    if tag == 9:
        Tag_mat = np.array([[-1,0,0,0.65],[0,1,0,0],[0,0,-1,-0.34],[0,0,0,1]])
        return  np.matmul(Tag_mat, matrix)
