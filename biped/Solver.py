import pinocchio as se3
from pinocchio.utils import *
from pinocchio.explog import log

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

class Solver: 
    """
    Class Solver: 
    Initialization takes 4 arguments : 
        - jointToMoveId : the id of the joint to move in a target pose (pose will be fetched in robot.data.oMi[jointToMoveId])
        - targetPose : an se3.SE3 of the target pose 
        - robot : the Robot used to solve
        - allowedDoFIds (optionnal) : a list of DoF the solver is allowed to move to solve the problem
            If not filled, all the DoF are used
    """
    def __init__(self, jointToMoveId, targetPose, robot, allowedDoFIds=None):
        self.jointToMoveId = jointToMoveId
        self.targetPose = targetPose
        self.robot = robot
        self.q = np.copy(self.robot.q)
        if allowedDoFIds is None: 
            self.allowedDoFIds = range(0, self.nq + 1)
        else: 
            self.allowedDoFIds = allowedDoFIds

    def cost(self, x): 
        q = self.q
        for (dof, num) in zip(self.allowedDoFIds, range(len(self.allowedDoFIds))):
            q[dof] = x[num]
        se3.forwardKinematics(robot.model, robot.data, q)
        jointToMovePose = robot.data.oMi[self.jointToMoveId]

        cost  = log(se3.SE3(jointToMovePose.rotation - self.targetPose.rotation, jointToMovePose.translation - self.targetPose.translation))

        return np.linalg.norm(cost.vector, ord=2)

    def minimize(self): 
        return fmin_bfgs(self.cost, self.q[self.allowedDoFIds[0]:self.allowedDoFIds[-1]+1])


if __name__ == "__main__": 
    robot = Robot() 


    for i in range(100): 
        q = robot.q
        target = se3.SE3.Random()

        robot.viewer.viewer.gui.addBox("world/target", .5, .5, .5, [0.7, 0.7, 0.7, 1.]) 
        robot.viewer.place("world/target", target)

        s_left = Solver(8, target, robot, allowedDoFIds=range(7, 15))
        q[7:15] = s_left.minimize()

        s_right = Solver(15, target, robot, allowedDoFIds=range(15, 21))
        q[15:22] = s_right.minimize()

        robot.display(q)

        raw_input()
        
    
