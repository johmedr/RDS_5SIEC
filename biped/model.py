

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

        self.TORSO_LX = 1
        self.TORSO_LY = 1
        self.TORSO_LZ = 1

        self.FEMUR_LX = 1
        self.FEMUR_LY = 1
        self.FEMUR_LZ = 1

        self.TIBIA_LX = 1
        self.TIBIA_LY = 1
        self.TIBIA_LZ = 1

        self.FOOT_LX = 1
        self.FOOT_LY = 1
        self.FOOT_LZ = 1

        self.JOINT_SPHERE_SIZE = 0.1

        self.COLOR = [red,green,blue,transparency] = [1,1,0.78,1.0]
        self.COLORRED = [1.0,0.0,0.0,1.0]

        self.viewer = Display()
        self.visuals = []
        self.model = se3.Model.BuildEmptyModel()
        self.createTorso(jointPlacement=se3.SE3(eye(3), np.matrix([0.,0.,self.FEMUR_LZ + self.TIBIA_LZ + 0.5 * self.TORSO_LZ])))
        self.createArm7DOF(rootId=1, prefix='left', jointPlacement=se3.SE3(eye(3), np.matrix([0.,0.5 * self.TORSO_LY,-0.5 * self.TORSO_LZ])))
        self.createArm7DOF(rootId=1, prefix='right', jointPlacement=se3.SE3(eye(3), np.matrix([0.,-0.5 * self.TORSO_LY,-0.5 * self.TORSO_LZ])))
        self.data = self.model.createData()
        self.q0 = zero(7)


    def createTorso(self, rootId=0, prefix='', jointPlacement=None): 
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        jointId = rootId

        name               = prefix+"torso"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = jointPlacement if jointPlacement!=None else se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelFreeFlyer(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())

        self.viewer.viewer.gui.addBox('world/'+prefix+'torso',0.5* self.TORSO_LX,0.5*self.TORSO_LY,0.5* self.TORSO_LZ,color)
        self.visuals.append( Visual('world/'+prefix+'torso',jointId,se3.SE3(eye(3),np.matrix([0.,0.,0.]))))

    # [TODO]
    def createArm7DOF(self,rootId=0,prefix='',jointPlacement=None):
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        jointId = rootId

        name               = prefix+"hip1"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = jointPlacement if jointPlacement!=None else se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRZ(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())

        name               = prefix+"hip2"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())

        name               = prefix+"hip3"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRX(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere3', 0.3,colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere3',jointId,se3.SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/'+prefix+'upperarm', .1,.1,.5,color)
        self.visuals.append( Visual('world/'+prefix+'upperarm',jointId,se3.SE3(eye(3),np.matrix([0.,0.,-.5]))))

        name               = prefix+"knee"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3),np.matrix( [0,0,-1.0] ))
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere4', 0.3,colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere4',jointId,se3.SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/'+prefix+'lowerarm', .1,.1,.5,color)
        self.visuals.append( Visual('world/'+prefix+'lowerarm',jointId,se3.SE3(eye(3),np.matrix([0.,0.,-.5]))))

        name               = prefix+"anckle1"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3),np.matrix( [0,0,-1.0] ))
        jointId = self.model.addJoint(jointId,se3.JointModelRX(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())

        name               = prefix+"anckle2"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere6', 0.3,colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere6',jointId,se3.SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/'+prefix+'foot', .3,.1,.1,color)
        self.visuals.append( Visual('world/'+prefix+'foot',jointId,se3.SE3(eye(3),np.matrix([.3,0.,0.]))))

        name               = prefix+"toe"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3), np.matrix( [0.6, 0, 0] ))
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
    
    def display(self,q):
        se3.forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()

if __name__ == "__main__": 
    robot = Robot()
    # q0 = rand(robot.model.nq)
    q0 = np.zeros(robot.model.nq)
    robot.display(q0)
    # q = rand(robot.model.nq)
    # delta_q = q-q0
    # nb_loop = 100
    # while True:
    #     for i in range(0,nb_loop):
    #         robot.display(q0+delta_q*i/nb_loop)
    #         q0 = q
    #         q = rand(robot.model.nq)
    #         delta_q = q-q0

        

