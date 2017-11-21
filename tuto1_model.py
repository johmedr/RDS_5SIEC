

from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as se3
import gepetto.corbaserver
from display import Display

class Visual:
    '''
    Class representing one 3D mesh of the robot, to be attached to a joint. The class contains:
    * the name of the 3D objects inside Gepetto viewer.
    * the ID of the joint in the kinematic tree to which the body is attached.
    * the placement of the body with respect to the joint frame.
    This class is only used in the list Robot.visuals (see below).
    '''
    def __init__(self,name,jointParent,placement):
        self.name = name                  # Name in gepetto viewer
        self.jointParent = jointParent    # ID (int) of the joint
        self.placement = placement        # placement of the body wrt joint, i.e. bodyMjoint
    def place(self,display,oMjoint):
        oMbody = oMjoint*self.placement
        display.place(self.name,oMbody,False)

class Robot:
    '''
    Define a class Robot with 7DOF (shoulder=3 + elbow=1 + wrist=3).
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
        self.viewer = Display()
        self.visuals = []
        self.model = se3.Model.BuildEmptyModel()
        self.createArm7DOF()
        self.data = self.model.createData()
        self.q0 = zero(7)

    def createArm7DOF(self,rootId=0,prefix='',jointPlacement=None):


	jointName          = "first_joint"                       # Name of joint.
	jointPlacement     = jointPlacement if jointPlacement!=None else se3.SE3.Identity()      # SE3 placement of the joint wrt chain init.
	parent             = rootId                                   # Index of the parent (0 is the universe).
	jointModel         = se3.JointModelRZ()                  # Type of the joint to be created.
	jointId = self.model.addJoint(parent,jointModel,jointPlacement,jointName)
	print('Model dimensions: {:d}, {:d}, {:d}'.format(self.model.nq,self.model.nv,self.model.nbodies))

	jointName          = "second_joint"                       # Name of joint.
        jointPlacement     = se3.SE3.Identity()      # SE3 pla$
        parent             = parent + 1                              # Index of the parent (0 is the universe$
        jointModel         = se3.JointModelRY()                  # Type of the joint to be created.
        jointId = self.model.addJoint(parent,jointModel,jointPlacement,jointName)
        print('Model dimensions: {:d}, {:d}, {:d}'.format(self.model.nq,self.model.nv,self.model.nbodies))

        jointName          = "third_joint"                       # Name of joint.
        jointPlacement     = se3.SE3.Identity()      # SE3 pla$
        parent             = parent + 1                              # Index of the parent (0 is the univ$
        jointModel         = se3.JointModelRX()                  # Type of the joint to be created.
        jointId = self.model.addJoint(parent,jointModel,jointPlacement,jointName)
        print('Model dimensions: {:d}, {:d}, {:d}'.format(self.model.nq,self.model.nv,self.model.nbodies))

        jointName          = "fourth_joint"                       # Name of joint.
        jointPlacement     = se3.SE3.Identity()      # SE3 pla$
        parent             = parent + 1                              # Index of the parent (0 is the univ$
        jointModel         = se3.JointModelRY()                  # Type of the joint to be created.
        jointId = self.model.addJoint(parent,jointModel,jointPlacement,jointName)
        print('Model dimensions: {:d}, {:d}, {:d}'.format(self.model.nq,self.model.nv,self.model.nbodies))

        jointName          = "fifth_joint"                       # Name of joint.
        jointPlacement     = se3.SE3.Identity()      # SE3 pla$
        parent             = parent + 1                              # Index of the parent (0 is the univ$
        jointModel         = se3.JointModelRX()                  # Type of the joint to be created.
        jointId = self.model.addJoint(parent,jointModel,jointPlacement,jointName)
        print('Model dimensions: {:d}, {:d}, {:d}'.format(self.model.nq,self.model.nv,self.model.nbodies))

        jointName          = "sixth_joint"                       # Name of joint.
        jointPlacement     = se3.SE3.Identity()      # SE3 pla$
        parent             = parent + 1                              # Index of the parent (0 is the univ$
        jointModel         = se3.JointModelRY()                  # Type of the joint to be created.
        jointId = self.model.addJoint(parent,jointModel,jointPlacement,jointName)
        print('Model dimensions: {:d}, {:d}, {:d}'.format(self.model.nq,self.model.nv,self.model.nbodies))

        jointName          = "seventh_joint"                       # Name of joint.
        jointPlacement     = se3.SE3.Identity()      # SE3 pla$
        parent             = parent + 1                              # Index of the parent (0 is the univ$
        jointModel         = se3.JointModelRY()                  # Type of the joint to be created.
        jointId = self.model.addJoint(parent,jointModel,jointPlacement,jointName)
        print('Model dimensions: {:d}, {:d}, {:d}'.format(self.model.nq,self.model.nv,self.model.nbodies))




    def display(self,q):
        se3.forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()


robot = Robot()
robot.display(robot.q0)
