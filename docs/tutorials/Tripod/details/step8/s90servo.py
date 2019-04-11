# -*- coding: utf-8 -*-
""" Sofa prefab for a S90 servo actuators with a default kinematic controller and visual model

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    Available prefab:
        - ServoMotor

    Available python object:
        - KinematicMotorController
        - ServoWheel
"""
import Sofa
from splib.numerics import RigidDof, Quat
from splib.objectmodel import *

from stlib.visuals import VisualModel
from stlib.solver import DefaultSolver
from stlib.scene import Scene, Node, get

@SofaPrefab
class ServoMotor(SofaObject):
    """A S90 servo motor

    This prefab is implementing a S90 servo motor.
    https://servodatabase.com/servo/towerpro/sg90

    The prefab ServoMotor is composed of:
    - a visual model
    - a mechanical model composed two rigids. One rigid is for the motor body
      while the other is implementing the servo rotating wheel.
    - a KinematicMotorController to compute from an input angle the new orientation of the
      servo wheel according to its parent frame.

    The prefab has the following parameters:
    - translation       to change default location of the servo (default [0.0,0.0,0.0])
    - rotation          to change default rotation of the servo (default [0.0,0.0,0.0,1])
    - scale             to change default scale of the servo (default 1)
    - doAddVisualModel  to control wether a visual model is added (default True)

    The prefab have the following property:
    - angle     use this to specify the angle of rotation of the servo motor

    Example of use in a Sofa scene:

    def createScene(root):
        ...
        servo = ServoMotor(root)

        ## Direct access to the components
        servo.node.angle = 1.0
        servo.dofs.showObjects = False

        ## Indirect access to the components
        get(servo.node, "dofs.showObjects").value = False
        get(servo.node, "servowheel.dofs.showObjects").value = False
    """

    def __init__(self, parent, translation=[0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0], scale=[1.0, 1.0, 1.0], doAddVisualModel=True):
        self.node = Node(parent, "ServoMotor")
        self.node.addNewData("angle",
                             "ServoMotor Properties",
                             "The angular position of the motor (in radians)", "double", 0.0)
        self.angle = self.node.findData("angle")

        self.dofs = self.node.createObject("MechanicalObject", size=1,
                                           name="dofs",
                                           translation=translation, rotation=rotation, scale3d=scale,
                                           template='Rigid3', showObject=True, showObjectScale=15)

        self.servowheel = ServoWheel(self.node)
        self.controller = KinematicMotorController(self.node, self.dofs, self.servowheel.dofs,
                                                   self.servowheel.node.map,
                                                   angleValue=self.angle.getLinkPath())

        if doAddVisualModel:
            self.addVisualModel()

    def addVisualModel(self):
        visual = self.node.createChild("VisualModel")
        visual.createObject("MeshSTLLoader", name="loader", filename="../data/mesh/SG90_servomotor.stl")
        visual.createObject("MeshTopology", src="@loader")
        visual.createObject("OglModel", color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        visual.createObject("RigidMapping", index=0)


class ServoWheel(SofaObject):
    def __init__(self, parentNode):
        self.node = Node(parentNode, "ServoWheel")
        self.dofs = self.node.createObject("MechanicalObject", size=1, template='Rigid3',
                                           showObject=True, showObjectScale=15, name="dofs")
        self.node.createObject("RigidRigidMapping", name="map", applyRestPosition=True)


class KinematicMotorController(Sofa.PythonScriptController):
    """
        This controller is in charge of transforming the angular 'positional' control of the
        of the ServoWheel.
    """

    def __init__(self, node, parentframe, target, dmap, angleValue):
        self.name = "controller"
        self.parentframe = parentframe
        self.node = node
        self.target = target
        self.dmap = dmap
        self.addNewData("angle", "Properties", "The angular position of the motor (in radians)", "double", angleValue)

    def applyAngleToServoWheel(self, angle):
        rigidparent = RigidDof(self.parentframe)
        rigidtarget = RigidDof(self.target)

        rigidtarget.copyFrom(rigidparent)
        self.dmap.initialPoints = [0.0, 0.0, 0.0]+list(Quat.createFromEuler([angle, 0,0]))

    def bwdInitGraph(self, root):
        self.applyAngleToServoWheel(self.angle)

    def onBeginAnimationStep(self, dt):
        self.applyAngleToServoWheel(self.angle)


def createScene(rootNode):
    from splib.animation import animate
    from splib.animation.easing import LinearRamp
    from splib.scenegraph import get

    from stlib.scene import MainHeader
    scene = Scene(rootNode)
    s = DefaultSolver(rootNode)

    # Test a assembly that also implements a KinematicMotorController
    # The angle of the KinematicMotorController is dynamically changed using a
    # animation function
    servomotor = ServoMotor(rootNode, translation=[2, 0, 0])
    def myAnimation(motorctrl, factor):
        motorctrl.angle = LinearRamp(-3.14/2, 3.14/2, factor)

    animate(myAnimation, {"motorctrl" : servomotor.node }, duration=1.0, mode="pingpong")
