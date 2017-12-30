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
    def __init__(self, jointToMoveId, targetPose, robot, allowedDoFIds=None, display=True, color=None):
        self.jointToMoveId = jointToMoveId
        self.targetPose = targetPose
        if jointToMoveId == 8:
            print self.targetPose
        self.robot = robot
        self.q = np.copy(self.robot.q)

        if allowedDoFIds is None: 
            self.allowedDoFIds = range(0, self.nq + 1)
        else: 
            self.allowedDoFIds = allowedDoFIds

        if display: 
            if color is None: 
                color = [0.7, 0.7, 0.7, 1.]
            robot.viewer.viewer.gui.addBox("world/target_j" + str(jointToMoveId), .1, .2, .3, color) 
            robot.viewer.place("world/target_j" + str(jointToMoveId), self.targetPose)

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

    T_ID = 1
    LF_ID = 8 
    RF_ID = 15

    RED = [1., 0., 0., 1.]
    GREEN = [0., 1., 0., 1.]
    BLUE = [0., 0., 1., 1.]
    
    q = np.copy(robot.display(robot.q0))
    offset_torso = robot.data.oMi[T_ID].translation - np2pin(np.array([0., 0., 1.]))

    target_left = se3.SE3(robot.data.oMi[LF_ID])
    target_right = se3.SE3(robot.data.oMi[RF_ID])


    for i in range(100): 
        target_torso = se3.SE3.Random()
        target_torso.translation += offset_torso

        s_torso = Solver(1, target_torso, robot, allowedDoFIds=range(0, 7), color=RED)
        q[0:7] = s_torso.minimize()

        # se3.forwardKinematics(robot.model, robot.data, q)

        s_left = Solver(8, target_left, robot, allowedDoFIds=range(7, 15), color=GREEN)
        q[7:15] = s_left.minimize()

        s_right = Solver(15, target_right, robot, allowedDoFIds=range(15, 21), color=BLUE)
        q[15:22] = s_right.minimize()

        robot.display(q)

        raw_input()
        
    
