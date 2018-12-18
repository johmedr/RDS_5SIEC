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

torso_pos = se3.SE3.Identity()
target_left = se3.SE3.Identity()
target_right = se3.SE3.Identity()

solver = Solver(T_ID, alternative=True, robot=robot, color=RED, ieqcons=(lambda x: x[6], lambda x: x[13]))
# solver = Solver(T_ID, alternative=True, robot=robot, color=RED)


q = robot.display(robot.q0)

q[2] -=0.9
q[4] = -1.0
q[6] = 1.5
q[11] = -1.
q[13] = 1.5

q = robot.display(q)
raw_input()


nSteps = 200
delta_t = zmpRef.get_trajectory_duration() / nSteps

time_index = footsteps.getIndexFromTime(0)
floor_pose = se3.SE3.Identity()
prev_flying='none'

target_left.translation = addHeightTo2DPos(footsteps.getLeftPosition(0), 0)
target_right.translation = addHeightTo2DPos(footsteps.getRightPosition(0), 0)
torso_z = float(robot.data.oMi[T_ID].translation[2])


def flying_foot_pos(actual, future, t, duration, time_start, H_PAS=0.5): 
	actual_x = actual[0]
	future_x = future[0]
	L_PAS = future_x - actual_x

	t -= time_start

	z0 = 4. * H_PAS / L_PAS

	x = actual_x  + (L_PAS * t**2 * (3*duration - 2 * t)) / duration**3 
	z = - z0 * (x - actual_x) * (x - future_x) / L_PAS 

	return np2pin(np.array([x, actual[1], z]))


for t_index in range(nSteps):

	t = t_index * delta_t

	zmp_pos = zmpRef(t)
	zmp_pos = addHeightTo2DPos(zmp_pos, torso_z)

	torso_pos.translation = zmp_pos
	flying_foot = footsteps.getPhaseType(t)

	if flying_foot == "left": 
		floor_foot = 'right'
		# target_left.translation = addHeightTo2DPos(footsteps.getLeftNextPosition(t), 0)
		target_left.translation = flying_foot_pos(footsteps.getLeftPosition(t), footsteps.getLeftNextPosition(t), t,footsteps.getPhaseDuration(t), footsteps.getPhaseStart(t))
		# target_right.translation = addHeightTo2DPos(footsteps.getRightPosition(t), 0)

	elif flying_foot == 'right': 
		floor_foot = 'left'
		# target_left.translation = addHeightTo2DPos(footsteps.getLeftPosition(t), 0)
		target_right.translation = flying_foot_pos(footsteps.getRightPosition(t), footsteps.getRightNextPosition(t), t, footsteps.getPhaseDuration(t), footsteps.getPhaseStart(t))

	elif flying_foot == 'none': 
		target_right.translation = addHeightTo2DPos(footsteps.getRightPosition(t), 0)
		target_left.translation = addHeightTo2DPos(footsteps.getLeftPosition(t), 0)
	# if flying_foot != prev_flying:
	# 	prev_flying = flying_foot
	# 	time_index = footsteps.getIndexFromTime(t)
	# 	if floor_foot == 'left':
	# 		target_left = se3.SE3(robot.data.oMi[LF_ID])
	# 	elif floor_foot == 'right': 
	# 		target_right = se3.SE3(robot.data.oMi[RF_ID])
	
	solver.set_target_pose(tar1=torso_pos, tar2=target_left, tar3=target_right)
	q = solver.minimize()

	q = robot.display(q)

# target_left = se3.SE3.Identity()
# # target_left.translation = np2pin(np.array([0.1, +0.3, -1.0]))


# target_right = se3.SE3.Identity()
# # target_right.translation = np2pin(np.array([0.7, -0.3, -2.0]))

# s_left = Solver(LF_ID - 1, target_left, robot, targetPoseRefJoint=T_ID, allowedDoFIds=range(3,10), color=BLUE, ieqcons=([lambda x: x[3]]))
# s_right = Solver(RF_ID, target_right, robot, targetPoseRefJoint=T_ID, allowedDoFIds=range(10,17), color=GREEN, ieqcons=([lambda x: x[3]]))

