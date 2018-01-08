import numpy as np
from pinocchio.utils import *

def np2pin(array):
    return np.matrix(array).T

def pin2np(ar):
    return ar.getA()[:,0]


def addHeightTo2DPos(list_xy, height): 
    pos = np.zeros(3)
    pos[0] = list_xy[0]
    pos[1] = list_xy[1]
    pos[2] = height
    return np2pin(pos)

RED = [1., 0., 0., 1.]
GREEN = [0., 1., 0., 1.]
BLUE = [0., 0., 1., 1.]