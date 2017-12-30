import pinocchio as se3
from pinocchio.utils import *
import numpy as np
from scipy.optimize import fmin_bfgs

from Robot import Robot
from display import Display
from utils import *


def rand_xyz(): 
    return np.random.rand(3)

def anim(robot, nb_loop, q0, q): 
    delta_q = (q - q0)
    print delta_q
    for i in range(0,nb_loop):
        robot.display(q0+delta_q*i/nb_loop)

if __name__ == "__main__": 
    robot = Robot() 
    
    q0 = robot.q
