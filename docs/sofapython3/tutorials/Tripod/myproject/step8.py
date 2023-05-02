# -*- coding: utf-8 -*-
"""
Step 8: Here we are showing how to setup the inverse control
"""
import Sofa
from parts.tutorial import *
from parts.tripod import Tripod
from parts.tripodcontroller import SerialPortController, SerialPortBridgeGeneric, InverseController, DirectController


def EffectorGoal(position):
    self = Sofa.Core.Node("Goal")
    # TODO: Define your effector goal here
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
    actuators = []
    for arm in arms:
        actuator = arm.ServoMotor.Articulation.addChild('actuator')
        actuators.append(actuator)
        actuator.activated = False
        # TODO: Add the actuator

    effector = freecenter.addChild("Effector")
    freecenter.dofs.showObject = True
    effector.activated = False
    actuators.append(effector)
    # TODO: Add the effector

    return actuators


def createScene(rootNode):
    from stlib3.scene import Scene
    scene = Scene(rootNode, gravity=[0., -9810., 0.], dt=0.01, iterative=False, plugins=["SofaSparseSolver", "SofaOpenglVisual", "SofaSimpleFem", "SoftRobots","SoftRobots.Inverse", 'SofaBoundaryCondition', 'SofaDeformable', 'SofaEngine', 'SofaGeneralRigid', 'SofaMiscMapping', 'SofaRigid', 'SofaGraphComponent', 'SofaGeneralAnimationLoop', 'SofaGeneralEngine'])

    # Adding contact handling
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')

    # Inverse Solver
    scene.addObject('FreeMotionAnimationLoop')
    # TODO: Replace the constraint solver with the inverse solver
    scene.Simulation.addObject('GenericConstraintSolver')

    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.Settings.mouseButton.stiffness = 10
    scene.VisualStyle.displayFlags = "showBehavior showCollision"

    tripod = scene.Modelling.addChild(Tripod())

    # Serial port bridge
    serial = SerialPortBridgeGeneric(rootNode)
    goalNode = EffectorGoal([0, 40, 0])
    scene.Modelling.addChild(goalNode)

    actuators = addInverseComponents(tripod.actuatedarms, tripod.RigidifiedStructure.FreeCenter, goalNode, False)

    # The real robot receives data from the 3 actuators
    # serialportctrl = scene.addObject(SerialPortController(scene, inputs=tripod.actuatedarms, serialport=serial))
    invCtr = scene.addObject(InverseController(scene, goalNode, actuators, tripod.ActuatedArm0.ServoMotor.Articulation.ServoWheel.RigidParts,
                                                tripod, serial,
                                                [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]))

    # The regular controller that is being used for the last 2 steps but with small additions
    scene.addObject(DirectController(scene, tripod.actuatedarms, invCtr))

    scene.Simulation.addChild(tripod)

