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

from s90servo import ServoMotor


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
                       translation=[0, 25, 0])

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
    prefabParameters = [
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0, 1.0, 1.0]}
    ]

    prefabData = [
        {'name': 'angleIn', 'group': 'ArmProperties', 'help': 'angle of rotation (in radians) of the arm',
         'type': 'float', 'default':0},
        {'name': 'angleOut', 'group': 'ArmProperties', 'type': 'float', 'help': 'angle of rotation (in radians) of '
                                                                                'the arm', 'default': 0}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)
        self.servoarm = None
        self.servomotor = None

    def init(self):
        self.servomotor = self.addChild(ServoMotor(name="ServoMotor", translation=self.translation.value,
                                                   rotation=self.rotation.value))
        self.servoarm = self.servomotor.Articulation.ServoWheel.addChild(ServoArm(name="ServoArm"))
        self.servoarm.setRigidMapping(self.ServoMotor.Articulation.ServoWheel.dofs.getLinkPath())

        # add a public attribute and connect it to the private one.
        self.ServoMotor.angleIn.setParent(self.angleIn)

        # connect the public attribute to the internal one.
        self.angleOut.setParent(self.ServoMotor.angleOut)


def createScene(rootNode):
    from splib3.animation import animate
    from stlib3.scene import Scene
    import math

    scene = Scene(rootNode, plugins=["ArticulatedSystemPlugin",
                                     "Sofa.Component.AnimationLoop", "Sofa.Component.Constraint.Lagrangian.Correction",
                                     "Sofa.Component.Constraint.Lagrangian.Solver",
                                     "Sofa.Component.Constraint.Projective", "Sofa.Component.IO.Mesh",
                                     "Sofa.Component.LinearSolver.Direct", "Sofa.Component.Mass", "Sofa.Component.SolidMechanics.Spring",
                                     "Sofa.Component.Topology.Container.Constant", "Sofa.Component.Visual",
                                     "Sofa.GL.Component.Rendering3D", "Sofa.GUI.Component",
                                     "Sofa.Component.Mapping.NonLinear",
                                     "Sofa.Component.StateContainer"], iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.VisualStyle.displayFlags = 'showBehavior'
    scene.dt = 0.01
    scene.gravity = [0., -9810., 0.]

    arm = scene.Modelling.addChild(ActuatedArm(name='ActuatedArm', translation=[0.0, 0.0, 0.0]))

    def myanimate(target, factor):
        target.angleIn.value = math.cos(factor * 2 * math.pi)

    animate(myanimate, {'target': arm}, duration=10., mode='loop')
    scene.Simulation.addChild(scene.Modelling)
