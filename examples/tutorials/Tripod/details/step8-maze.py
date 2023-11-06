# -*- coding: utf-8 -*-
"""
Step 8-maze: Here we are showing how to use the inverse control to solve a maze
"""
import Sofa
from tutorial import *
from tripod import Tripod
from tripodcontroller import SerialPortController, SerialPortBridgeGeneric, InverseController, DirectController
from maze import Maze, Sphere
from mazecontroller import MazeController
from splib3.interface import serialport


def EffectorGoal(node, position):
    goal = node.addChild('Goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, threshold=1e-12, tolerance=1e-10)
    goal.addObject('MechanicalObject', name='goalMO', template='Rigid3', position=position+[0., 0., 0., 1.], showObject=True, showObjectScale=10)
    goal.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
    goal.addObject('UncoupledConstraintCorrection', compliance=[1e-10] * 7)
    return goal


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
                                                minAngle=-1.0, maxAngle=1.0, maxAngleVariation=0.1)

    effector = freecenter.addChild("Effector")
    effector.activated = False
    actuators.append(effector)
    if goalNode is None:
        effector.addObject('PositionEffector', name='effector', template='Rigid3',
                               useDirections=[1, 1, 1, 0, 0, 0],
                               indices=0, effectorGoal=[10, 40, 0], limitShiftToTarget=True,
                               maxShiftToTarget=5)
    elif use_orientation:
        effector.addObject('PositionEffector', name='effectorY', template='Rigid3',
                            useDirections=[0, 1, 0, 0, 0, 0],
                            indices=0, effectorGoal=goalNode.goalMO.getLinkPath() + '.position',
                            limitShiftToTarget=True, maxShiftToTarget=5)
        effector.addObject('PositionEffector', name='effectorRXRZ', template='Rigid3', 
                            useDirections=[0, 0, 0, 1, 0, 1],
                            indices=0, effectorGoal=goalNode.goalMO.getLinkPath() + '.position')
    else:
        effector.addObject('PositionEffector', name='effector', template='Rigid3',
                               useDirections=[1, 1, 1, 0, 0, 0],
                               indices=0, effectorGoal=goalNode.goalMO.getLinkPath() + '.position',
                               limitShiftToTarget=True, maxShiftToTarget=5)
    return actuators


def createScene(rootNode):
    from stlib3.scene import Scene
    import json

    pluginList = ["ArticulatedSystemPlugin",
                  "Sofa.Component.AnimationLoop",
                  "Sofa.Component.Collision.Detection.Algorithm",
                  "Sofa.Component.Collision.Detection.Intersection",
                  "Sofa.Component.Collision.Geometry",
                  "Sofa.Component.Collision.Response.Contact",
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
                  "SoftRobots.Inverse",
                  "Sofa.Component.Mapping.Linear",
                  "Sofa.Component.Mapping.NonLinear"]

    scene = Scene(rootNode, gravity=[0., -9810, 0.], dt=0.01, iterative=False, plugins=pluginList)
    ContactHeader(rootNode, alarmDistance=15, contactDistance=0.5, frictionCoef=0)
    scene.removeObject(scene.GenericConstraintSolver)

    goalNode = EffectorGoal(rootNode, [0, 30, 0])

    # Open maze planning from JSON file
    data = json.load(open('mazeplanning.json'))
    goalNode.addObject(MazeController(goalNode, data["anglePlanningTable"], False))

    # Adding contact handling
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')

    # Inverse Solver
    scene.addObject('QPInverseProblemSolver', name='QP', printLog=False, epsilon=0.01)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    tripod = scene.Modelling.addChild(Tripod())
    actuators = addInverseComponents(tripod.actuatedarms, tripod.RigidifiedStructure.FreeCenter, goalNode, True)
    maze = tripod.RigidifiedStructure.FreeCenter.addChild(Maze())
    maze.addObject("RigidMapping", index=0)
    scene.Simulation.addChild(Sphere())

    # Serial port bridge
    serial = SerialPortBridgeGeneric(rootNode, serialport=serialport.getDevicePort('Arduino', method='manufacturer'))

    # The real robot receives data from the 3 actuators
    # serialportctrl = scene.addObject(SerialPortController(scene, inputs=tripod.actuatedarms, serialport=serial))
    invCtr = scene.addObject(InverseController(scene, goalNode, actuators, tripod.ActuatedArm0.ServoMotor.Articulation.ServoWheel.RigidParts,
                                                tripod, serial,
                                                [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]))

    # The regular controller that is being used for the last 2 steps but with small additions
    scene.addObject(DirectController(scene, tripod.actuatedarms, invCtr))

    scene.Simulation.addChild(tripod)
