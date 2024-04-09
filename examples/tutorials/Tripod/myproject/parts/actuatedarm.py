# -*- coding: utf-8 -*-
""" ActuatedArm for the tripod robot.

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    Available prefab:
        - ActuatedArm
        - ServoArm
"""
import Sofa
from stlib3.visuals import VisualModel

from parts.s90servo import ServoMotor


class ServoArm(Sofa.Prefab):
    """ServoArm is a reusable sofa model of a servo arm for the S90 servo motor

       Parameters:
            parent:        node where the ServoArm will be attached
            mappingInput:  the rigid mechanical object that will control the orientation of the servo arm
            indexInput: (int) index of the rigid the ServoArm should be mapped to
    """

    prefabData = [
        {'name': 'mappingInputLink', 'type': 'string',
         'help': 'the rigid mechanical object that will control the orientation of the servo arm', 'default': ''},
        {'name': 'indexInput', 'type': 'int', 'help': 'index of the rigid the ServoArm should be mapped to',
         'default': 1}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.addObject('MechanicalObject',
                       name='dofs',
                       size=1,
                       template='Rigid3',
                       showObject=True,
                       showObjectScale=5,
                       translation2=[0, 25, 0])

    def setRigidMapping(self, path):
        self.addObject('RigidMapping', name='mapping', input=path, index=self.indexInput.value)

        visual = self.addChild(VisualModel(visualMeshPath='data/mesh/SG90_servoarm.stl', translation=[0., -25., 0.],
                                           color=[1., 1., 1., 0.75]))
        visual.OglModel.writeZTransparent = True
        visual.addObject('RigidMapping', name='mapping')


class ActuatedArm(Sofa.Prefab):
    """ActuatedArm is a reusable sofa model of a S90 servo motor and the tripod actuation arm.
           Parameters:
             - translation the position in space of the structure
             - eulerRotation the orientation of the structure

           Structure:
           Node : {
                name : 'ActuatedArm'
                MechanicalObject     // Rigid position of the motor
                ServoMotor           // The s90 servo motor with its actuated wheel
                ServoArm             // The actuation arm connected to ServoMotor.ServoWheel
            }
    """
    prefabData = [
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0, 1.0, 1.0]}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.servomotor = self.addChild(
            ServoMotor(name="ServoMotor", translation=self.translation.value, rotation=self.rotation.value))
        self.servoarm = self.servomotor.Articulation.ServoWheel.addChild(ServoArm(name="ServoArm"))
        self.servoarm.setRigidMapping(self.ServoMotor.Articulation.ServoWheel.dofs.getLinkPath())

        # add a public attribute and connect it to the private one.
        self.addData(name='angleIn', group='ArmProperties', help='angle of rotation (in radians) of the arm',
                     type='float', value=0)
        self.ServoMotor.getData('angleIn').setParent(self.getData('angleIn'))

        # add a public attribute and connect it to the internal one.
        self.addData(name='angleOut', group='ArmProperties', help='angle of rotation (in radians) of the arm',
                     type='float', value=0)
        self.getData('angleOut').setParent(self.ServoMotor.getData('angleOut'))


def createScene(rootNode):
    from splib3.animation import animate
    from stlib3.scene import Scene
    import math

    scene = Scene(rootNode, iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.VisualStyle.displayFlags = 'showBehavior'
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]

    arm = scene.Simulation.addChild(ActuatedArm(name='ActuatedArm', translation=[0.0, 0.0, 0.0]))

    def myanimate(target, factor):
        target.angleIn.value = math.cos(factor * 2 * math.pi)

    animate(myanimate, {'target': arm}, duration=10., mode='loop')
