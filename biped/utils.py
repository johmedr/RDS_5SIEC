import numpy as np
from pinocchio.utils import *

def np2pin(array):
    return np.matrix(array).T

def pin2np(ar):
    return ar.getA()[:,0]