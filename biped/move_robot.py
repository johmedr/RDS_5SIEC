from ZmpRef import ZmpRef
from Robot import Robot 
from Solver import Solver
from FootSteps import FootSteps
from utils import *
import pinocchio as se3

footsteps = FootSteps( [0.0,-0.5] , [0.0,0.5] )
footsteps.addPhase( .3, 'none' )
footsteps.addPhase( .7, 'left' , [1.2,+0.5] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', [2.4,-0.5] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'left' , [3.6,+0.5] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', [4.8,-0.5] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'left' , [6.,+0.5] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', [7.2,-0.5] )
footsteps.addPhase( .5, 'none' )

robot = Robot()
zmpRef = ZmpRef(footsteps)


T_ID = robot.torsoId
LF_ID = robot.leftLegLastJointId
RF_ID = robot.rightLegLastJointId

torso_z = robot.data.oMi[T_ID].translation[2]
torso_pos = se3.SE3.Identity()

s_torso = Solver(T_ID, torso_pos, robot, allowedDoFIds=range(0, 3), color=RED)

q = robot.display(robot.q0)

nSteps = 1000
delta_t = zmpRef.get_trajectory_duration() / nSteps

for t in range(nSteps):

	zmp_pos = zmpRef(t * delta_t)
	zmp_pos = addHeightTo2DPos(zmp_pos, torso_z)

	torso_pos.translation = zmp_pos
	s_torso.set_target_pose(torso_pos)

	q[0:3] = s_torso.minimize()
	q = robot.display(q)




# target_left = se3.SE3.Identity()
# # target_left.translation = np2pin(np.array([0.1, +0.3, -1.0]))


# target_right = se3.SE3.Identity()
# # target_right.translation = np2pin(np.array([0.7, -0.3, -2.0]))

# s_left = Solver(LF_ID - 1, target_left, robot, targetPoseRefJoint=T_ID, allowedDoFIds=range(3,10), color=BLUE, ieqcons=([lambda x: x[3]]))
# s_right = Solver(RF_ID, target_right, robot, targetPoseRefJoint=T_ID, allowedDoFIds=range(10,17), color=GREEN, ieqcons=([lambda x: x[3]]))

