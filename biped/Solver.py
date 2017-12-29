import pinocchio as se3
from pinocchio.utils import *
import numpy as np
from scipy.optimize import fmin_bfgs

from copy import copy

from Robot import Robot
from display import Display
from utils import *


def cost(a, b): 
    return np.linalg.norm(a.translation - b.translation, ord=2)# + np.linalg.norm(a.rotation - b.rotation, ord=2)

def constraint_not_move_torso(q):
    return constraint_not_move_from_pos_for_range(q, 0, 2)

def constraint_not_move_left_leg(q): 
    return constraint_not_move_from_pos_for_range(q, 3, 9)

def constraint_not_move_right_leg(q): 
    return constraint_not_move_from_pos_for_range(q, 10, 16)

class Solver:
    def __init__(self, robot, show_dest=False): 
        self.robot = robot

        self.leftFootDest = robot.get_left_foot_pos()
        self.rightFootDest = robot.get_right_foot_pos()
        
        self.torsoDest = robot.get_torso_pos()
        
        self.show_dest = show_dest
        if self.show_dest: 
            self.display = robot.viewer
            radius = 0.2
            height = 0.2
            self.display.viewer.gui.addCylinder('world/cyl_tors', radius,height, [1., 0., 0., 1.])
            self.display.viewer.gui.addCylinder('world/cyl_left', radius,height, [0., 1., 0., 1.])
            self.display.viewer.gui.addCylinder('world/cyl_right', radius,height, [0., 0., 1., 1.])


    def set_left_foot_target_pose(self, targetPose):
        if isinstance(targetPose, np.ndarray): 
            if targetPose.shape == (3,): 
                self.leftFootDest = se3.SE3(eye(3), np.matrix(targetPose))
        elif isinstance(targetPose, se3.SE3): 
            self.leftFootDest = targetPose
        else:
            raise TypeError()

        if self.show_dest: 
            self.display.place('world/cyl_left',self.leftFootDest)

    def cost_left_foot(self, x): 
        q = robot.q
        q[3:10] = x
        pos = robot.get_left_foot_pos(q)
        # return np.linalg.norm(pos.translation - self.leftFootDest.translation, ord=2)# + np.linalg.norm(pos.rotation - self.leftFootDest.rotation, ord=2)
        # return np.linalg.norm(pos.translation - self.leftFootDest.translation, ord=2) + np.linalg.norm(pos.rotation - self.leftFootDest.rotation, ord=2)
        return cost(pos, self.leftFootDest)

    def minimize_left(self):
        return fmin_bfgs(self.cost_left_foot, self.robot.q[3:10])

    def set_right_foot_target_pose(self, targetPose):
        if isinstance(targetPose, np.ndarray): 
            if targetPose.shape == (3,): 
                self.rightFootDest = se3.SE3(-eye(3), np.matrix(targetPose))
        elif isinstance(targetPose, se3.SE3): 
            self.rightFootDest = targetPose
        else:
            raise TypeError()

        if self.show_dest: 
            self.display.place('world/cyl_right', self.rightFootDest)

    def cost_right_foot(self, x): 
        q = robot.q
        q[10:17] = x
        pos = robot.get_right_foot_pos(q)
        # return np.linalg.norm(pos.translation - self.rightFootDest.translation, ord=2)# + np.linalg.norm(pos.rotation - self.rightFootDest.rotation, ord=2)
        # return np.linalg.norm(pos.translation - self.rightFootDest.translation, ord=2) + np.linalg.norm(pos.rotation - self.rightFootDest.rotation, ord=2)
        return cost(pos, self.rightFootDest)

    def minimize_right(self):
        return fmin_bfgs(self.cost_right_foot, self.robot.q[10:17])

    def set_torso_target_pose(self, targetPose):
        if isinstance(targetPose, np.ndarray): 
            if targetPose.shape == (3,): 
                self.torsoDest = se3.SE3(eye(3), np.matrix(targetPose))
        elif isinstance(targetPose, se3.SE3): 
            self.torsoDest = targetPose
        else:
            raise TypeError()

        if self.show_dest: 
            self.display.place('world/cyl_tors',se3.SE3(eye(3), self.torsoDest.translation + self.robot.torsoInitPosition.translation))

        # self.torsoControlLeftFoot = self.robot.get_left_foot_pos()
        # self.torsoControlRightFoot = self.robot.get_right_foot_pos()

# opti full config
    # def cost_torso(self, q):  
    #     pos = robot.get_torso_pos(q)
    #     left_foot = robot.get_left_foot_pos(q)
    #     right_foot = robot.get_right_foot_pos(q)
    #     # return np.linalg.norm(pos.translation - self.torsoDest.translation, ord=2)# + np.linalg.norm(pos.rotation - self.torsoDest.rotation, ord=2)
    #     return cost(pos, self.torsoDest) + cost(left_foot, self.torsoControlLeftFoot) + cost(right_foot, self.torsoControlRightFoot)

    def cost_torso(self, x): 
        q = robot.q
        q[0:3] = x
        pos = robot.get_torso_pos(q)
        # return np.linalg.norm(pos.translation - self.torsoDest.translation, ord=2)# + np.linalg.norm(pos.rotation - self.torsoDest.rotation, ord=2)
        return cost(pos, self.torsoDest)

    def minimize_torso(self):
        # return fmin_bfgs(self.cost_torso, self.robot.q) opti full config
        # left_foot_pose = robot.get_left_foot_pos() 

        # left_foot_pose.translation -= self.torsoDest.translation

        solver.set_left_foot_target_pose(pin2np(robot.get_left_foot_pos().translation))
        solver.set_right_foot_target_pose(pin2np(robot.get_right_foot_pos().translation))
        q = robot.q

        # q[0:3] = fmin_bfgs(self.cost_torso, self.robot.q[0:3])
        q[0:3] = pin2np(self.torsoDest.translation)
        
        robot.move(q)

        q[3:10] = self.minimize_left()
        q[10:17] = self.minimize_right()

        print self.torsoDest

        print self.robot.get_left_foot_pos()
        print self.leftFootDest

        print self.robot.get_right_foot_pos()
        print self.rightFootDest

        return q




# def set_joint_destination(self, jointId, position):
#     self.pdest = pdest
#     self.display.place('world/cylin',se3.SE3(eye(3), np.matrix( pdest ) ))

 #    def cost(self, q): 
 #        if type(q) != np.ndarray : 
 #            print "wrong type"
 #        se3.forwardKinematics(robot.model,robot.data,q)
 #        p = pin2np(robot.data.oMi[-1].translation)
 #        dist = np.linalg.norm(p - self.pdest, ord=2) 
	# return dist
    
    def minimize(self, pdest):
        self.set_pdest(pdest)
        return fmin_bfgs(self.cost, self.q0) 

def rand_xyz(): 
    return np.random.rand(3)

def anim(robot, nb_loop, q0, q): 
    delta_q = (q - q0)
    print delta_q
    for i in range(0,nb_loop):
        robot.display(q0+delta_q*i/nb_loop)

if __name__ == "__main__": 
    robot = Robot() 
    solver  = Solver(robot, show_dest=True) 

    q0 = robot.q

    while(True):
        # for i in range(5):
        #     solver.set_left_foot_target_pose(np.array([np.random.rand(), np.random.rand(), np.random.rand()]))
        #     solver.set_right_foot_target_pose(np.array([np.random.rand(), -np.random.rand(), np.random.rand()]))

        #     q_left = solver.minimize_left()
        #     q_right = solver.minimize_right()

        #     q = robot.q
        #     q[3:10] = q_left
        #     q[10:17] = q_right

        #     # anim(robot, 100, q0, q)
        #     robot.display(q)
        #     q0 = q

        #     raw_input()

        q0 = np.zeros(robot.model.nq)
        robot.display(q0)


        raw_input()


        for i in range(5):
            solver.set_torso_target_pose((np.random.rand(3) - np.ones(3)) * 0.5)
            q = solver.minimize_torso()

            # anim(robot, 100, q0, q)
            robot.display(q)
            q0 = q

            raw_input()

    # q0 = solver.q0
    
    # while(True):
    #     qmin = Solverr.minimize(rand_xyz())
    #     anim(robot, 100, q0, qmin)
    #     q0 = qmin
