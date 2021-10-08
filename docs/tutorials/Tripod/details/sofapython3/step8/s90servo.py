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
from splib3.numerics import RigidDof, Quat
from splib3.objectmodel import *

from stlib3.visuals import VisualModel
from stlib3.solver import DefaultSolver
from stlib3.scene import Scene
dirPath = os.path.dirname(os.path.abspath(__file__))

def ServoWheel(parent, showWheel=True):

    servowheel = parent.addChild('ServoWheel')
    servowheel.addObject('MechanicalObject', template='Rigid3', name='dofs', position=[[0., 0., 0., 0., 0., 0., 1.]],
                            showObject=showWheel, showObjectScale=10)
    servowheel.addObject("RigidRigidMapping", name="map", applyRestPosition=True)

    return servowheel


class ServoMotor(Sofa.Prefab):
    '''A S90 servo motor

    This prefab is implementing a S90 servo motor.
    https://servodatabase.com/servo/towerpro/sg90

    The prefab ServoMotor is composed of:
    - a visual model
    - a mechanical model composed two rigids. One rigid is for the motor body
      while the other is implementing the servo rotating wheel.

    The prefab has the following parameters:
    - translation           to change default location of the servo (default [0.0,0.0,0.0])
    - rotation              to change default rotation of the servo (default [0.0,0.0,0.0,1])
    - scale                 to change default scale of the servo (default 1)
    - showServo             to control wether a visual model of the motor is added (default True)
    - showWheel             to control wether the rotation axis of the motor is displayed (default False)

    The prefab have the following property:
    - angle         use this to specify the angle of rotation of the servo motor
    - angleLimits   use this to set a min and max value for the servo angle rotation
    - position      use this to specify the position of the servo motor

    Example of use in a Sofa scene:

    def addScene(root):
        ...
        servo = ServoMotor(root)

        ## Direct access to the components
        servo.angle.value = 1.0
    '''
    properties = [
        {'name':'name',                'type':'string', 'help':'Node name',                   'default':'ServoMotor'},
        {'name':'rotation',            'type':'Vec3d',  'help':'Rotation',                    'default':[0.0, 0.0, 0.0]},
        {'name':'translation',         'type':'Vec3d',  'help':'Translation',                 'default':[0.0, 0.0, 0.0]},
        {'name':'scale3d',             'type':'Vec3d',  'help':'Scale 3d',                    'default':[1.0, 1.0, 1.0]},
        {'name':'showServo',           'type':'bool',   'help':'',                            'default':True},
        {'name':'showWheel',           'type':'bool',   'help':'',                            'default':False}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):

        self.addData(name="angle",
                     group="ServoMotor Properties",
                     help="The angular position of the motor (in radians)", type="double", default=0.0)

        # One position (rigid): first one for the servo body, second for the servo wheel
        self.dofs = self.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[[0., 0., 0., 0., 0., 0., 1.]],
                       translation=list(self.translation.value),rotation=list(self.rotation.value),scale3d=list(self.scale3d.value))

        # ServoBody and ServoWheel objects with visual
        self.servowheel = ServoWheel(self, showWheel=self.showWheel)
        self.controller = self.addObject(KinematicMotorController(self, self.dofs, self.servowheel.dofs,
                                                                   self.servowheel.map))

        if self.showServo:
            self.addVisualModel()

    def addVisualModel(self):
        visual = self.addChild("VisualModel")
        visual.addObject("MeshSTLLoader", name="loader", filename="../../data/mesh/SG90_servomotor.stl")
        visual.addObject("MeshTopology", src="@loader")
        visual.addObject("OglModel", color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        visual.addObject("RigidMapping", index=0)

class KinematicMotorController(Sofa.Core.Controller):
    """
        This controller is in charge of transforming the angular 'positional' control of the
        of the ServoWheel.
    """
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "controller"
        self.node = args[0]
        self.parentframe = args[1]
        self.target = args[2]
        self.dmap = args[3]

    def applyAngleToServoWheel(self, angle):
        rigidparent = RigidDof(self.parentframe)
        rigidtarget = RigidDof(self.target)

        rigidtarget.copyFrom(rigidparent)
        self.dmap.initialPoints.value = [[0.0, 0.0, 0.0]+list(Quat.createFromEuler([angle, 0.0,0.0]))]

    def onAnimateBeginEvent(self, e):
        self.applyAngleToServoWheel(self.node.angle.value)


def createScene(rootNode):
    from splib3.animation import animate
    from splib3.animation.easing import LinearRamp
    from splib3.scenegraph import get

    scene = Scene(rootNode, plugins=['SofaConstraint', 'SofaGeneralRigid', 'SofaRigid', 'SofaOpenglVisual'])
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('DefaultAnimationLoop')

    # Test a assembly that also implements a KinematicMotorController
    # The angle of the KinematicMotorController is dynamically changed using a
    # animation function
    servomotor = rootNode.Simulation.addChild(ServoMotor(rootNode.Simulation, translation=[2, 0, 0]))
    servomotor.ServoWheel.dofs.showObject = True
    def myAnimation(motorctrl, factor):
        motorctrl.angle = LinearRamp(-3.14/2, 3.14/2, factor)

    animate(myAnimation, {"motorctrl" : servomotor }, duration=10.0, mode="pingpong")
