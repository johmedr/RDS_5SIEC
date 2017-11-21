# Create a 7DOF manipulator robot and moves it along a constant (random) velocity during 10secs.
from pinocchio.utils import *
from numpy.linalg import pinv,norm
from robot_arm import Robot
import time

# Create a 7DOF robot.
robot = Robot()

# Hide the floor.
robot.viewer.viewer.gui.setVisibility('world/floor','OFF')

# Move the robot during 10secs at velocity v.
v = rand(robot.model.nv)
q = np.copy(robot.q0)
dt = 5e-3
for i in range(10000):
    q += v*dt
    robot.display(q)
    time.sleep(dt)
