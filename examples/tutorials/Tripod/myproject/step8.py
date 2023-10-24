# -*- coding: utf-8 -*-
"""
Step 8: Here we are showing how to setup the inverse control
"""
import Sofa
from tutorial import *
from tripod import Tripod
from tripodcontroller import SerialPortController, SerialPortBridgeGeneric, InverseController, DirectController
from splib3.interface import serialport


def EffectorGoal(position):
    self = Sofa.Core.Node('Goal')
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
            self.time = self.time + self.dt

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

    pluginList = ["ArticulatedSystemPlugin",
                  "Sofa.Component.AnimationLoop",
                  "Sofa.Component.Collision.Geometry",
                  "Sofa.Component.Constraint.Lagrangian.Correction",
                  "Sofa.Component.Constraint.Projective",
                  "Sofa.Component.Engine.Select",
                  "Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.LinearSolver.Iterative",
                  "Sofa.Component.Mapping.MappedMatrix",
                  "Sofa.Component.Mass",
                  "Sofa.Component.SolidMechanics.FEM.Elastic",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Constant",
                  "Sofa.Component.Topology.Container.Dynamic",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.GUI.Component",
                  "SoftRobots",
                  "SoftRobots.Inverse", "Sofa.Component.Mapping.Linear", "Sofa.Component.Mapping.NonLinear"]

    scene = Scene(rootNode, gravity=[0., -9810., 0.], dt=0.01, iterative=False, plugins=pluginList)

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
    serial = SerialPortBridgeGeneric(rootNode, serialport=serialport.getDevicePort('Arduino', method='manufacturer'))

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
    invCtr = scene.addObject(
        InverseController(scene, goalNode, actuators, tripod.ActuatedArm0.ServoMotor.Articulation.ServoWheel.RigidParts,
                          tripod, serial,
                          [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]))

    # The regular controller that is being used for the last 2 steps but with small additions
    scene.addObject(DirectController(scene, tripod.actuatedarms, invCtr))

    scene.Simulation.addChild(tripod)
