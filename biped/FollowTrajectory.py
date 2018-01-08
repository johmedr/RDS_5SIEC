from Robot import Robot
from Solver import Solver
from ZmpRef import ZmpRef
from FootSteps import FootSteps
from utils import *

import pinocchio as se3
import numpy as np

class FollowTrajectory: 
    def __init__(self, robot, foosteps, height=None, dynamicWalk=False):
        assert(isinstance(robot, Robot))
        assert(isinstance(foosteps, FootSteps))

        self.robot = robot
        q = self.robot.q0
        q[10] = 0.1
        q[17] = 0.1
        self.robot.display(q)

        self.foosteps = foosteps
        self.zmpRef = ZmpRef(foosteps)

        if height is not None: 
            self.height = height
        else: 
            self.height = self.robot.data.oMi[self.robot.torsoId].translation[2] - 0.5

        self.refFootHeight = self.robot.data.oMi[self.robot.leftLegLastJointId - 1].translation[2]
        
        self.torsoTarget = se3.SE3.Identity() 
        self.leftTarget = se3.SE3.Identity()
        self.rightTarget = se3.SE3.Identity()

        self.extractTargets(t = 0)

        self.solverTorso = Solver(
            self.robot.torsoId, 
            self.torsoTarget, 
            self.robot, 
            allowedDoFIds=range(0, 7), 
            color=RED, 
            eqcons=([lambda x: 
                (np.sqrt(x[3]**2 + x[4]**2 + x[5]**2 + x[6]*2) - 1)])
            )

        self.solverLeft = Solver(
                    self.robot.leftLegLastJointId - 1, 
                    self.leftTarget, 
                    self.robot, 
                    allowedDoFIds=range(7, 15), 
                    color=BLUE, 
                    ieqcons=([lambda x: x[3]])
                    )

        self.solverRight = Solver(
                    self.robot.rightLegLastJointId - 1, 
                    self.rightTarget, 
                    self.robot, 
                    allowedDoFIds=range(15, 21), 
                    color=GREEN, 
                    ieqcons=([lambda x: x[3]])
                    )


    def extractTargets(self, t): 
        self.torsoTarget.translation = addHeightTo2DPos( self.zmpRef(t), self.height )

        self.leftTarget.translation = addHeightTo2DPos( self.foosteps.getLeftPosition(t), 0)
        self.rightTarget.translation = addHeightTo2DPos( self.foosteps.getRightPosition(t), 0)

    def moveToTargets(self):
        q = np.copy(self.robot.q)

        q[0:7] = self.solverTorso.minimize()
        q = self.robot.display(q, disp=False)

        q[7:15] = self.solverLeft.minimize()
        q[15:21] = self.solverRight.minimize()
        self.robot.display(q)

    def follow_trajectory_static(self, t): 
        self.extractTargets(t)
        self.moveToTargets()

    def follow_trajectory_dynamic(self): 
        pass


if __name__ == "__main__": 
    robot = Robot()

    footsteps = FootSteps( [0.0,-0.5], [0.0,0.5])
    footsteps.addPhase( .3, 'none' )
    footsteps.addPhase( .7, 'left' , [0.5,+0.5] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'right', [1.0,-0.5] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'left' , [1.5,+0.5] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'right', [2.0,-0.5] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'left' , [2.5,+0.5] )
    footsteps.addPhase( .1, 'none' )
    footsteps.addPhase( .7, 'right', [3.0,-0.5] )
    footsteps.addPhase( .5, 'none' )

    traj = FollowTrajectory(robot,footsteps)

    nSteps = 1000
    delta_t = footsteps.getTrajectoryDuration() / nSteps

    t = 0
    for i in range(nSteps): 
        t += delta_t

        traj.follow_trajectory_static(t)
