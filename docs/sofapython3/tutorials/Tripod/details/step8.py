# -*- coding: utf-8 -*-
"""
Step 8: Here we are showing how to setup the inverse control
"""
import Sofa
from tutorial import *
from tripod import Tripod
from tripodcontroller import SerialPortController, SerialPortBridgeGeneric, InverseController, DirectController


def EffectorGoal(position):
    self = Sofa.Core.Node('Goal')
    self.addObject('EulerImplicitSolver', firstOrder=True)
    self.addObject('CGLinearSolver', iterations=100, threshold=1e-5, tolerance=1e-5)
    self.addObject('MechanicalObject', name='goalMO', template='Rigid3', position=position+[0., 0., 0., 1.], showObject=True, showObjectScale=10)
    self.addObject('UncoupledConstraintCorrection')

    spheres = self.addChild('Spheres')
    spheres.addObject('MechanicalObject', name='mo', position=[[0, 0, 0],  [10, 0, 0],   [0, 10, 0],   [0, 0, 10]])
    spheres.addObject('SphereCollisionModel', radius=5, group=1)
    spheres.addObject('RigidMapping')
    return self

class GoalController(Sofa.Core.Controller):
    """This controller moves the goal position when the inverse control is activated
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "GoalController"
        self.activated = False
        self.time = 0
        self.dy = 0.1
        goalNode = args[1]
        self.mo = goalNode.goalMO
        self.dt = goalNode.getRoot().dt.value

    def onKeyPressed(self, key):
        if key == Key.I:
            self.activated = True

    def onAnimateBeginEvent(self, e):
        if self.activated:
            self.time = self.time+self.dt

        if self.time >= 1:
            self.time = 0;
            self.dy = -self.dy

        pos = [self.mo.position[0][0], self.mo.position[0][1], self.mo.position[0][2]]
        pos[1] += self.dy
        self.mo.position = [[pos[0], pos[1], pos[2], 0, 0, 0, 1]]


def addInverseComponents(arms, freecenter, goalNode, use_orientation):
    actuators=[]
    for arm in arms:
        actuator = arm.ServoMotor.Articulation.addChild('actuator')
        actuators.append(actuator)
        actuator.activated = False
        actuator.addObject('JointActuator', name='JointActuator', template='Vec1',
                                                index=0, applyForce=True,
                                                minAngle=-1.5, maxAngle=1.5, maxAngleVariation=0.008)

    effector = freecenter.addChild("Effector")
    effector.activated = False
    actuators.append(effector)
    if goalNode is None:
        effector.addObject('PositionEffector', name='effector', template='Rigid3',
                               useDirections=[1, 1, 1, 0, 0, 0],
                               indices=0, effectorGoal=[10, 40, 0], limitShiftToTarget=True,
                               maxShiftToTarget=5)
    elif use_orientation:
        effector.addObject('PositionEffector', name='effector', template='Rigid3',
                               useDirections=[0, 1, 0, 1, 0, 1],
                               indices=0, effectorGoal=goalNode.goalMO.position.getLinkPath(),
                               limitShiftToTarget=True, maxShiftToTarget=5)
    else:
        effector.addObject('PositionEffector', name='effector', template='Rigid3',
                               useDirections=[1, 1, 1, 0, 0, 0],
                               indices=0, effectorGoal=goalNode.goalMO.position.getLinkPath(),
                               limitShiftToTarget=True, maxShiftToTarget=5)
    return actuators


def createScene(rootNode):
    from stlib3.scene import Scene
    scene = Scene(rootNode, gravity=[0., -9810., 0.], dt=0.01, iterative=False, plugins=["SofaSparseSolver", "SofaOpenglVisual", "SofaSimpleFem", "SoftRobots","SoftRobots.Inverse", 'SofaBoundaryCondition', 'SofaDeformable', 'SofaEngine', 'SofaGeneralRigid', 'SofaMiscMapping', 'SofaRigid', 'SofaGraphComponent', 'SofaGeneralAnimationLoop', 'SofaGeneralEngine'])

    # Adding contact handling
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')

    # Inverse Solver
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('QPInverseProblemSolver', name='QP', printLog=False)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.Settings.mouseButton.stiffness = 10
    scene.VisualStyle.displayFlags = "showBehavior showCollision"

    tripod = scene.Modelling.addChild(Tripod())

    # Serial port bridge
    serial = SerialPortBridgeGeneric(rootNode)

    # Choose here to control position or orientation of end-effector
    orientation = False
    if orientation:
        # inverse in orientation
        goalNode = EffectorGoal([0, 50, 50])
    else:
        # inverse in position
        goalNode = EffectorGoal([0, 40, 0])
    scene.Modelling.addChild(goalNode)

    actuators = addInverseComponents(tripod.actuatedarms, tripod.RigidifiedStructure.FreeCenter, goalNode, orientation)

    # The real robot receives data from the 3 actuators
    # serialportctrl = scene.addObject(SerialPortController(scene, inputs=tripod.actuatedarms, serialport=serial))
    invCtr = scene.addObject(InverseController(scene, goalNode, actuators, tripod.ActuatedArm0.ServoMotor.Articulation.ServoWheel.RigidParts,
                                                tripod, serial,
                                                [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]))

    # The regular controller that is being used for the last 2 steps but with small additions
    scene.addObject(DirectController(scene, tripod.actuatedarms, invCtr))

    scene.Simulation.addChild(tripod)

    # Temporary additions to have the system correctly built in SOFA
    # Will no longer be required in SOFA v22.06
    scene.Simulation.addObject('MechanicalMatrixMapper',
                                 name="deformableAndFreeCenterCoupling",
                                 template='Vec3,Rigid3',
                                 object1=tripod["RigidifiedStructure.DeformableParts.dofs"].getLinkPath(),
                                 object2=tripod["RigidifiedStructure.FreeCenter.dofs"].getLinkPath(),
                                 nodeToParse=tripod["RigidifiedStructure.DeformableParts.MechanicalModel"].getLinkPath())

    for i in range(3):
        scene.Simulation.addObject('MechanicalMatrixMapper',
                                   name="deformableAndArm{i}Coupling".format(i=i),
                                   template='Vec1,Vec3',
                                   object1=tripod["ActuatedArm" + str(i) + ".ServoMotor.Articulation.dofs"].getLinkPath(),
                                   object2=tripod["RigidifiedStructure.DeformableParts.dofs"].getLinkPath(),
                                   skipJ2tKJ2=True,
                                   nodeToParse=tripod["RigidifiedStructure.DeformableParts.MechanicalModel"].getLinkPath())
