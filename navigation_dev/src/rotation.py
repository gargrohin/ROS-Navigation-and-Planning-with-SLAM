import math
import numpy as np



def getAnglesFromRotationMatrix(Rmat):

    theta = math.acos(((Rmat[0][0] + Rmat[1][1] + Rmat[2][2]) - 1) / 2)

    multi = 1 / (2 * math.sin(theta))
    rx = multi * (Rmat[2][1] - Rmat[1][2]) * theta
    ry = multi * (Rmat[0][2] - Rmat[2][0]) * theta
    rz = multi * (Rmat[1][0] - Rmat[0][1]) * theta
    return rx, ry, rz


def dummy(Rmat):

    rx = math.atan(Rmat[2][1]/Rmat[2][2])
    ry = math.asin(Rmat[2][0])
    rz = math.atan(Rmat[1][0]/Rmat[0][0])
    return rx, ry, rz