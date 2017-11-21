import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *

path = '/home/student/models/'
urdf = path + 'ur_description/urdf/ur5_gripper.urdf'
pkgs = [ path, ] # paths where to find the model meshes

robot = RobotWrapper(urdf,pkgs)   # Load urdf model
                                  # The robot is loaded with the basis fixed to the world
robot.initDisplay(loadModel=True) # Setup the viewer to display the robot

NQ = robot.model.nq               # model configuration size (6)
NV = robot.model.nv               # model configuration velocity size (6)

q  = rand(NQ)                     # Set up an initial configuration
robot.display(q)

vq = zero(NV)
vq[3] = 1                         # Set up a constant robot speed

from time import sleep
for i in range(10000):            # Move the robot with constant velocity
    q += vq/100
    robot.display(q)
    sleep(.01)
