import pinocchio as se3
from pinocchio.utils import *
from pinocchio.explog import log

import numpy as np
from scipy.optimize import fmin_bfgs, fmin_slsqp

from Robot import Robot
from display import Display
from utils import *

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
    def __init__(self, jointToMoveId, targetPose, robot, allowedDoFIds=None, eqcons=(), ieqcons=(), display=True, color=None):
        self.jointToMoveId = jointToMoveId
        self.targetPose = targetPose
        self.robot = robot
        self.q = np.copy(self.robot.q)

        if allowedDoFIds is None: 
            self.allowedDoFIds = range(0, self.robot.model.nq)
        else: 
            self.allowedDoFIds = allowedDoFIds

        self.eqcons = eqcons
        self.ieqcons = ieqcons

        self.display = display 
        if self.display: 
            if color is None: 
                color = [0.7, 0.7, 0.7, 1.]
            robot.viewer.viewer.gui.addBox("world/target_j" + str(jointToMoveId), .1, .2, .3, color) 
            robot.viewer.place("world/target_j" + str(jointToMoveId), self.targetPose)

    def set_target_pose(self, targetPose): 
        self.targetPose = targetPose
        if self.display: 
            self.robot.viewer.place("world/target_j" + str(self.jointToMoveId), self.targetPose)

    def raw_cost(self, x=None): 
        jointToMovePose = robot.data.oMi[self.jointToMoveId]

        # cost = log(se3.SE3(jointToMovePose.rotation - self.targetPose.rotation, jointToMovePose.translation - self.targetPose.translation))
        cost = log(se3.SE3(jointToMovePose.rotation.transpose() * self.targetPose.rotation, self.targetPose.translation - jointToMovePose.translation))
        # cost = log(jointToMovePose.inverse().act(self.targetPose))

        # print vars(cost * cost)
        return np.linalg.norm(cost.vector, ord='fro')

    def cost(self, x): 
        q = self.q
        for (dof, num) in zip(self.allowedDoFIds, range(len(self.allowedDoFIds))):
            q[dof] = x[num]
        se3.forwardKinematics(robot.model, robot.data, q)

        return self.raw_cost(x)


    def minimize(self): 
        if self.eqcons is () and self.ieqcons is (): 
            return fmin_bfgs(
                        self.cost, 
                        self.q[self.allowedDoFIds[0]:self.allowedDoFIds[-1]+1], 
                        maxiter=10)
        else: 
            return fmin_slsqp(
                        self.cost, self.q[self.allowedDoFIds[0]:self.allowedDoFIds[-1]+1], 
                        eqcons=self.eqcons, ieqcons=self.ieqcons)


if __name__ == "__main__": 
    robot = Robot() 

    T_ID = 1
    LF_ID = 8 
    RF_ID = 15

    RED = [1., 0., 0., 1.]
    GREEN = [0., 1., 0., 1.]
    BLUE = [0., 0., 1., 1.]
    
    q = np.copy(robot.display(robot.q0))
    offset_torso = robot.data.oMi[T_ID].translation

    target_left = se3.SE3(robot.data.oMi[LF_ID])

    target_right = se3.SE3(robot.data.oMi[RF_ID])
    target_right.translation += np2pin(np.array([0., 0., 0.5]))

    s_left = Solver(LF_ID , target_left, robot, allowedDoFIds=range(7, 15), color=BLUE, ieqcons=([lambda x: x[3]]))
    s_right = Solver(RF_ID - 1, target_right, robot, allowedDoFIds=range(15,21), color=GREEN)
    q[15:21] = s_right.minimize()
    q = robot.display(q)

    L_PAS = 1.
    H_PAS = 0.5
    T = 100.
    z0 = 4. * H_PAS / L_PAS

    x0 = pin2np(target_left.translation)[0] - L_PAS/2
    x1 = pin2np(target_left.translation)[0] + L_PAS/2

    for t in range(int(T)): 
        x = x0 + (L_PAS * t**2 * (3*T - 2 * t)) / T**3 
        z = - z0 * (x - x0) * (x - x1) / L_PAS
        target_left.translation = np.matrix([x, 0.5, z + 0.])
        s_left.set_target_pose(target_left)
        q[7:15] = s_left.minimize() 
        q = robot.display(q)
        
