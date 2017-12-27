import pinocchio as se3
from pinocchio.utils import *
import numpy as np
from scipy.optimize import fmin_bfgs
from tp2_model import Robot
from display import Display

def np2pin(array):
    return np.matrix(array).T

def pin2np(ar):
    return ar.getA()[:,0]

class Solver:
    def __init__(self, robot, pdest = np.zeros(3), q0 = None): 
        self.robot = robot
        self.display = robot.viewer
        self.pdest = pdest

        radius = 0.2
        height = 0.2
        color = [0., 1., 0., 1.]
        self.display.viewer.gui.addCylinder('world/cylin', radius,height,color)
        
        if q0 == None: 
            self.q0 = np.zeros(self.robot.model.nq)
        else: 
            assert(type(q0) == np.ndarray)
            self.q0 = q0

    def set_pdest(self, pdest):
        self.pdest = pdest
        self.display.place('world/cylin',se3.SE3(eye(3), np.matrix( pdest ) ))

    def cost(self, q): 
        if type(q) != np.ndarray : 
            print "wrong type"
        se3.forwardKinematics(robot.model,robot.data,q)
        p = pin2np(robot.data.oMi[-1].translation)
        dist = np.linalg.norm(p - self.pdest, ord=2) 
	return dist
    
    def minimize(self, pdest):
        self.set_pdest(pdest)
        return fmin_bfgs(self.cost, self.q0) 

def rand_dest(): 
    return 2*np.random.rand(3)

def anim(robot, nb_loop, q0, q):
    delta_q = (q - q0)
    print delta_q
    for i in range(0,nb_loop):
        robot.display(q0+delta_q*i/nb_loop)

if __name__ == "__main__": 
    robot = Robot() 
    solver  = Solver(robot) 
    q0 = solver.q0
    
    while(True):
        qmin = solver.minimize(rand_dest())
        anim(robot, 100, q0, qmin)
        q0 = qmin
