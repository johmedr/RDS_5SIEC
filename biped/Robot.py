from exceptions import TypeError, AttributeError

from pinocchio.utils import *
from numpy.linalg import pinv,norm
import numpy as np
from scipy.optimize import fmin_bfgs, minimize

from pinocchio.explog import exp,log
import pinocchio as se3
import gepetto.corbaserver

from Display import Display
from Visual import Visual
from utils import *
from constraints import *

STATE_REST = 0
STATE_FIRST_STEP = 1
STATE_STEADY = 2
STATE_FLY = 3
STATE_LAST_STEP = 4
STATE_READY = 5

RIGHT_LEG = 0
LEFT_LEG = 1

class Robot:
    '''
    Define a class Robot with 7DOF (hip=3 + elbow=1 + anckle=3).
    The configuration is nq=7. The velocity is the same.
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot, each element of the list being
    an object Visual (see above).

    See tp1.py for an example of use.
    '''

    def __init__(self):

        self.TORSO_LX = float(1)
        self.TORSO_LY = float(1)
        self.TORSO_LZ = float(2)

        self.FEMUR_LX = float(0.2)
        self.FEMUR_LY = float(0.2)
        self.FEMUR_LZ = float(1)

        self.TIBIA_LX = float(0.2)
        self.TIBIA_LY = float(0.2)
        self.TIBIA_LZ = float(1)

        self.FOOT_LX = float(0.8)
        self.FOOT_LY = float(0.4)
        self.FOOT_LZ = float(0.2)

        self.JOINT_SPHERE_SIZE = float(0.2)

        self.COLOR = [red,green,blue,transparency] = [1,1,0.78,1.0]
        self.COLORRED = [1.0,0.0,0.0,1.0]

        self.viewer = Display()
        self.visuals = []
        self.model = se3.Model.BuildEmptyModel()

        self.torsoId = self.createTorso(jointPlacement=se3.SE3(eye(3), np.matrix([0.,0.,0.5*self.JOINT_SPHERE_SIZE + self.FOOT_LZ + self.FEMUR_LZ + self.TIBIA_LZ + 0.5 * self.TORSO_LZ])))
        self.leftLegLastJointId = self.createLeg(rootId=self.torsoId, prefix='left', jointPlacement=se3.SE3(eye(3), np.matrix([0.,0.5 * self.TORSO_LY,-0.5 * self.TORSO_LZ])))
        self.rightLegLastJointId = self.createLeg(rootId=self.torsoId, prefix='right', jointPlacement=se3.SE3(eye(3), np.matrix([0.,-0.5 * self.TORSO_LY,-0.5 * self.TORSO_LZ])))
        
        self.data = self.model.createData()

        self.q0 = np.zeros(self.model.nq)
        self.q = self.q0
        self.display(self.q0)

        self.torsoInitPosition = self.data.oMi[self.torsoId]
        self.leftFootInitPosition = self.data.oMi[self.leftLegLastJointId]
        self.rightFootInitPosition = self.data.oMi[self.rightLegLastJointId]

        self.state = STATE_REST
        self.flyingLeg = RIGHT_LEG

        self.cost = None 

    def fake_move(self, q): 
        tmp_model = self.model
        tmp_data = self.data
        se3.forwardKinematics(tmp_model, tmp_data, q)
        return tmp_model, tmp_data

    def get_pos(self, jointId, q=None): 
        if q is None:
            return self.data.oMi[self.torsoId]
        elif isinstance(q, np.ndarray):
            model, data = self.fake_move(q)
            return data.oMi[jointId]

    def get_torso_pos(self, q=None): 
        return self.get_pos(self.torsoId, q)

    def get_left_foot_pos(self, q=None):
        return self.get_pos(self.leftLegLastJointId, q)

    def get_right_foot_pos(self, q=None): 
        return self.get_pos(self.rightLegLastJointId, q)

    # def get_default_constraints(self): 
    #     cons = ()
    #     """
    #     Constraint hip rotation on z
    #     """
    #     cons += (
    #             {'type': 'eq', 
    #               'fun': lambda x: np.array(x[3])}, 
    #             {'type': 'eq', 
    #               'fun': lambda x: np.array(x[10])}
    #             )
    #     """
    #     Constraint knees rotation > 0
    #     """
    #     cons += (
    #             {'type': 'ineq', 
    #               'fun': lambda x: np.array(x[6])}, 
    #             {'type': 'ineq', 
    #               'fun': lambda x: np.array(x[13])}
    #             )
    #     return cons

    # def get_state_constraints(self): 
    #     cons = ()
    #     if self.state is STATE_REST: 
    #         cons += () 

    # def get_constraint_foot_dont_move(self, side): 
    #     if side == "left":
    #         lastJointId = self.leftLegLastJointId
    #         initPosition = self.leftFootInitPosition
    #     elif side == "right": 
    #         lastJointId = self.rightLegLastJointId
    #         initPosition = self.rightFootInitPosition
    #     else: 
    #         raise AttributeError

    #     def constraint_foot_dont_move(q): 
    #         tmp_model = self.model
    #         tmp_data = self.data
    #         # Optimize this with only one call to fwd kinematics by keeping a optimization model and data aside
    #         se3.forwardKinematics(tmp_model, tmp_data, q)
    #         return pin2np(tmp_data.oMi[lastJointId].translation - initPosition.translation)

    #     return {'type': 'eq', 
    #              'fun' : constraint_foot_dont_move}

    # def cost_rest(self, q):
    #     tmp_model = self.model
    #     tmp_data = self.data

    #     # target = self.data.oMi[self.torsoInitPosition].translation[:, 0]


    #     # target_right = self.data.oMi[self.rightLegLastJointId - 7]

    #     target = self.torsoInitPosition.translation[:,0]
    #     target[2] -= .5
    #     self.viewer.viewer.gui.addCylinder('world/cylin', 1,0.5,self.COLORRED)
    #     self.viewer.place('world/cylin',se3.SE3(eye(3), np.matrix( target ) ))
    #     se3.forwardKinematics(tmp_model, tmp_data, q)
    #     return np.linalg.norm(tmp_data.oMi[self.torsoId].translation - target)

    # def test_cost(self, x): 
    #     tmp_model = self.model 
    #     tmp_data = self.data

    #     # target = self.data.oMi[self.torsoInitPosition].translation[:, 0]


    #     # target_right = self.data.oMi[self.rightLegLastJointId - 7]

    #     target = self.leftFootInitPosition.translation[:,0]
    #     target[0] = self.FOOT_LY * 0.75 
    #     target[1] = 0.5
    #     target[2] = 1.5 + self.FOOT_LZ
    #     q = self.q0 
    #     q[4] = x[0]
    #     q[6] = x[1]
    #     q[8] = x[2]
    #     q[11] = x[0]
    #     q[13] = x[1]
    #     q[15] = x[2]

    #     self.viewer.viewer.gui.addCylinder('world/cylin', 0.2,0.2,self.COLORRED)
    #     self.viewer.place('world/cylin',se3.SE3(eye(3), np.matrix( target ) ))
    #     se3.forwardKinematics(tmp_model, tmp_data, q)
    #     return np.linalg.norm(tmp_data.oMi[self.leftLegLastJointId].translation - target)

    # def test_solve(self): 
    #     return fmin_bfgs(self.test_cost, np.zeros(3))

    # def cost_ready(self, q):
    #     pass

    # def cost_first_step(self, q):
    #     pass

    # def cost_steady(self, q):
    #     pass

    # def cost_fly(self, q):
    #     pass

    # def cost_last_step(self, q):
    #     pass

    # def switchState(self):
    #     # ... 
    #     if self.state is STATE_REST: 
    #         self.cost = self.cost_rest

    #     elif self.state is STATE_FIRST_STEP: 
    #         pass 
    #         # self.cost = self.cost_first_step... 
    #     elif self.state is STATE_STEADY: 
    #         # Move CoM to other foot
    #         pass
    #     elif self.state is STATE_FLY:
    #         pass
    #     elif self.state is STATE_LAST_STEP: 
    #         pass

    # def solve(self): 
    #     if self.state is STATE_REST: 
    #         self.cost = self.cost_rest
    #         constraints = (self.get_constraint_foot_dont_move("left"), 
    #                        self.get_constraint_foot_dont_move("right"))
    #         constraints += DEFAULT_CONSTRAINTS 
    #         constraints += MOVE_PLANE_XZ_CONSTRAINTS
    #         constraints += SYMETRIC_CONSTRAINTS
    #         # constraints += TORSO_CONSTRAINTS_TZ
    #         q = minimize(fun=self.cost, x0=self.q0, method='SLSQP', constraints=constraints)
    #         return q


    def createTorso(self, rootId=0, prefix='', jointPlacement=None): 
        color   = [red,green,blue,transparency] = [1,1,0.78,0.5]
        colorred = [1.0,0.0,0.0,1.0]

        jointId = rootId

        name               = prefix+"torso"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = jointPlacement if jointPlacement!=None else se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelTranslation(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Identity(),se3.SE3.Identity())

        self.viewer.viewer.gui.addBox('world/'+prefix+'torso',0.5* self.TORSO_LX,0.5*self.TORSO_LY,0.5* self.TORSO_LZ,color)
        self.visuals.append( Visual('world/'+prefix+'torso',jointId,se3.SE3(eye(3),np.matrix([0.,0.,0.]))))

        return jointId


    def createLeg(self,rootId=0,prefix='',jointPlacement=None):
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        jointId = rootId

        name               = prefix+"hip1"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = jointPlacement if jointPlacement!=None else se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRZ(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Identity(),se3.SE3.Identity())

        name               = prefix+"hip2"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Identity(),se3.SE3.Identity())

        name               = prefix+"hip3"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRX(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Identity(),se3.SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere3', self.JOINT_SPHERE_SIZE, colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere3',jointId,se3.SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/'+prefix+'femur', 0.5*self.FEMUR_LX,0.5*self.FEMUR_LY,0.5*self.FEMUR_LZ,color)
        self.visuals.append( Visual('world/'+prefix+'femur',jointId,se3.SE3(eye(3),np.matrix([0.,0.,-0.5*self.FEMUR_LZ]))))

        name               = prefix+"knee"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3),np.matrix( [0,0,-self.FEMUR_LZ] ))
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Identity(),se3.SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere4', self.JOINT_SPHERE_SIZE, colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere4',jointId,se3.SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/'+prefix+'lowerarm', 0.5*self.TIBIA_LX,0.5*self.TIBIA_LY,0.5*self.TIBIA_LZ,color)
        self.visuals.append( Visual('world/'+prefix+'lowerarm',jointId,se3.SE3(eye(3),np.matrix([0.,0.,-0.5* self.TIBIA_LZ]))))

        name               = prefix+"anckle1"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3),np.matrix( [0,0,-self.TIBIA_LZ] ))
        jointId = self.model.addJoint(jointId,se3.JointModelRX(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Identity(),se3.SE3.Identity())

        name               = prefix+"anckle2"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Identity(),se3.SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere6', self.JOINT_SPHERE_SIZE, colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere6',jointId,se3.SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/'+prefix+'foot', 0.5*self.FOOT_LX,0.5*self.FOOT_LY,0.5*self.FOOT_LZ,color)
        self.visuals.append( Visual('world/'+prefix+'foot',jointId,se3.SE3(eye(3),np.matrix([0.25*self.FOOT_LX,0.,-self.JOINT_SPHERE_SIZE]))))

        name               = prefix+"toe"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix( [0.75*self.FOOT_LX, 0, 0] ))
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Identity(),se3.SE3.Identity())

        return jointId

    
    def display(self,q):
        se3.forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()
        self.q = q

if __name__ == "__main__": 
    robot = Robot()
    # q0 = 2* rand(robot.model.nq) - np.ones(robot.model.nq)
    q0 = np.zeros(robot.model.nq)
    # q0[0:7] = np.zeros((7,1))
    # print q0
    # q0 = np.zeros(robot.model.nq)
    robot.display(q0)
    print robot.leftLegLastJointId
    print robot.rightLegLastJointId
    print robot.leftFootInitPosition
    print robot.rightFootInitPosition
    # print robot.solve()
    # print robot.test_solve()

    # q0[2] = -0.5
    q0[4] = -0.75 
    q0[6] = 1.5
    q0[8] = -0.75 

    q0[11] = -0.75
    q0[13] = 1.5
    q0[15] = -0.75
    robot.display(q0)
    # print robot.solve()
    # x = robot.test_solve()
    q0[4] = x[0]
    q0[6] = x[1]
    q0[8] = x[2]

    q0[11] = x[0]
    q0[13] = x[1]
    q0[15] = x[2]

    robot.display(q0)

    # robot.display(q0)
    # print se3.centerOfMass(robot.model, robot.data, q0)
    # # print se3.computeJacobians(robot.model, robot.data, q0)
    # print se3.jacobianCenterOfMass(robot.model, robot.data, q0)

    # print se3.crba(robot.model, robot.data, q0)


    # q = 2* rand(robot.model.nq) - np.ones(robot.model.nq)
    # delta_q = q-q0
    # nb_loop = 1000
    # while True:
    #     for i in range(0,nb_loop):
    #         robot.display(q0+delta_q*i/nb_loop)
    #         q0 = q
    #         q = 2* rand(robot.model.nq) - np.ones(robot.model.nq)
    #         delta_q = q-q0;

        

