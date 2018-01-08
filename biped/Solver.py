import pinocchio as se3
from pinocchio.utils import *
from pinocchio.explog import log

import numpy as np
from scipy.optimize import fmin_bfgs, fmin_slsqp

from Robot import Robot
from display import Display
from utils import *

from copy import copy

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
    def __init__(self, jointToMoveId, targetPose, robot, targetPoseRefJoint=0, allowedDoFIds=None, eqcons=(), ieqcons=(), display=True, color=None):
        self.jointToMoveId = jointToMoveId
        self.robot = robot
        self.q = np.copy(self.robot.q)

        self.targetPoseRefJoint = targetPoseRefJoint

        self.targetPose = se3.SE3(targetPose)

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
            # robot.viewer.place("world/target_j" + str(jointToMoveId), self.targetPose)

        self.set_target_pose(targetPose)


    def set_target_pose(self, targetPose): 
        self.targetPose = targetPose
        reference = self.robot.data.oMi[self.targetPoseRefJoint]

        self.targetPose.translation += reference.translation

        if self.display: 
            self.robot.viewer.place("world/target_j" + str(self.jointToMoveId), self.targetPose)



    def raw_cost(self, x=None): 
        jointToMovePose = self.robot.data.oMi[self.jointToMoveId]

        # cost = log(se3.SE3(jointToMovePose.rotation - self.targetPose.rotation, jointToMovePose.translation - self.targetPose.translation))
        # cost = log(se3.SE3(jointToMovePose.rotation.transpose() * self.targetPose.rotation, self.targetPose.translation - jointToMovePose.translation))
        cost = log(jointToMovePose.inverse() * self.targetPose)

        # print vars(cost * cost)
        return np.linalg.norm(cost.vector, ord='fro')
        # return np.linalg.norm(pin2np(jointToMovePose.translation - self.targetPose.translation))

    def cost(self, x): 
        q = np.copy(self.q)
        for (dof, num) in zip(self.allowedDoFIds, range(len(self.allowedDoFIds))):
            q[dof] = x[num]
        se3.forwardKinematics(self.robot.model, self.robot.data, q)

        return self.raw_cost(x)


    def minimize(self): 
        if self.eqcons is () and self.ieqcons is (): 
            return fmin_bfgs(
                        self.cost, 
                        self.q[self.allowedDoFIds[0]:self.allowedDoFIds[-1]+1], 
                        maxiter=100)
        else: 
            return fmin_slsqp(
                        self.cost, self.q[self.allowedDoFIds[0]:self.allowedDoFIds[-1]+1], 
                        eqcons=self.eqcons, ieqcons=self.ieqcons, 
                        iter=10)


if __name__ == "__main__": 
    robot = Robot() 

    T_ID = robot.torsoId
    LF_ID = robot.leftLegLastJointId
    RF_ID = robot.rightLegLastJointId

    RED = [1., 0., 0., 1.]
    GREEN = [0., 1., 0., 1.]
    BLUE = [0., 0., 1., 1.]
    
    q = np.copy(robot.display(robot.q0))

    q[0] = 1
    q[1] = -1
    q[2] -= 0.5


    q[6] = 0.1
    q[13] = 0.1

    robot.display(q)

    torso_offset_z = robot.data.oMi[T_ID].translation

    raw_input()

    target_left = se3.SE3.Identity()
    # target_left.translation = np2pin(np.array([0.1, +0.3, -1.0]))


    target_right = se3.SE3.Identity()
    target_right.translation = np2pin(np.array([0.7, -0.3, -2.0]))

    s_left = Solver(LF_ID - 1, target_left, robot, targetPoseRefJoint=T_ID, allowedDoFIds=range(3,10), color=BLUE, ieqcons=([lambda x: x[3]]))
    s_right = Solver(RF_ID, target_right, robot, targetPoseRefJoint=T_ID, allowedDoFIds=range(10,17), color=GREEN, ieqcons=([lambda x: x[3]]))

    # q = robot.display(q)

    q[3:10] = s_left.minimize()
    q[10:17] = s_right.minimize()
    robot.display(q)

    L_PAS = 1.2
    H_PAS = 0.5
    T = 200.
    z0 = 4. * H_PAS / L_PAS

    x0 = - L_PAS/2
    x1 = L_PAS/2

    for t in range(int(T)): 
        
        x = x0  + (L_PAS * t**2 * (3*T - 2 * t)) / T**3 
        z = - z0 * (x - x0) * (x - x1) / L_PAS 

        target_left.translation = np.matrix([x, 0.5, z - torso_offset_z[2] + robot.FOOT_LZ + robot.JOINT_SPHERE_SIZE * 0.5])

        s_left.set_target_pose(target_left)
        q[3:10] = s_left.minimize() 
        q = robot.display(q)
        
