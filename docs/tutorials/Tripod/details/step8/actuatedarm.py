# -*- coding: utf-8 -*-
""" ActuatedArm for the tripod robot.

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    Available prefab:
        - ActuatedArm
        - ServoArm
"""
from splib.numerics import Transform, vec3, RigidDof
from splib.objectmodel import *
from stlib.visuals import VisualModel
from stlib.components import addOrientedBoxRoi
from s90servo import ServoMotor
import Sofa

@SofaPrefab
class ServoArm(object):
    def __init__(self, parent, mappingInput, name="ServoArm", indexInput=0):
        """ServoArm is a reusable sofa model of a servo arm for the S90 servo motor

           Parameters:
                parent:        node where the ServoArm will be attached
                mappingInput:  the rigid mechanical object that will control the orientation of the servo arm
                indexInput (int)

           Structure:
           Node : {
                name : "ActuatedArm"
                MechanicalObject     // Rigid position of the motor
                ServoMotor           // The s90 servo motor with it actuated wheel
                ServoArm             // The actuation arm connected to ServoMotor/ServoWheel
            }
        """
        self.node = parent.createChild(name)
        self.node.createObject("MechanicalObject",
                               name="dofs",
                               size=1,
                               template='Rigid3',
                               showObject=True,
                               showObjectScale=15)

        self.node.createObject('RigidRigidMapping',
                               name="mapping", input=mappingInput, index=indexInput)

        # visual = VisualModel(self.node, '../data/mesh/SG90_servoarm.stl')
        # visual.createObject('RigidMapping', name="mapping")


@SofaPrefab
class ActuatedArm(object):
    """ActuatedArm is a reusable sofa model of a S90 servo motor and the tripod actuation arm.
           Parameters:
             - translation the position in space of the structure
             - eulerRotation the orientation of the structure
             - attachingTo (MechanicalObject)    a rest shape forcefield will constraint the object
                                                 to follow arm position
           Structure:
           Node : {
                name : "ActuatedArm"
                MechanicalObject     // Rigid position of the motor
                ServoMotor           // The s90 servo motor with its actuated wheel
                ServoArm             // The actuation arm connected to ServoMotor.ServoWheel
            }
    """

    def __init__(self, parent, name="ActuatedArm",
                 translation=[0.0, 0.0, 0.0], eulerRotation=[0.0, 0.0, 0.0], attachingTo=None):

        self.node = parent.createChild(name)
        r = Transform(translation, eulerRotation=eulerRotation)

        self.node.createObject("MechanicalObject", name="dofs", size=1, position=r.toSofaRepr(),
                               template='Rigid3', showObject=True, showObjectScale=15)

        servomotor = ServoMotor(self.node)
        servomotor.createObject("RigidRigidMapping", name="mapping")
        ServoArm(self.node, servomotor.ServoWheel.dofs)

        if attachingTo is not None:
            constraint = self.addConstraint(attachingTo.dofs.getData("rest_position"), translation, eulerRotation)
            attachingTo.createObject('RestShapeSpringsForceField',
                                     points=constraint.BoxROI.getData("indices"),
                                     external_rest_shape=constraint.dofs,
                                     stiffness='1e12')

    def addFrameConstraint(self, frameDof, indice):
        self.node.Box.createObject("RigidRigidMapping", name="mapping",
                                   output=self.ServoMotor.ServoWheel.dofs.getLinkPath(), index=indice,
                                   input=frameDof.getLinkPath())

    def addBox(self, position, translation, eulerRotation):
        constraint = self.node.createChild("Box")
        o = addOrientedBoxRoi(constraint, position=position,
                              translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                              eulerRotation=eulerRotation, scale=[45, 15, 30])
        o.drawBoxes = False
        o.init()

    def addConstraint(self, position, translation, eulerRotation):
        constraint = self.node.createChild("Constraint")
        addOrientedBoxRoi(constraint, position=position,
                          translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                          eulerRotation=eulerRotation, scale=[45, 15, 30])

        constraint.createObject("TransformEngine", input_position="@BoxROI.pointsInROI",
                               translation=translation, rotation=eulerRotation, inverse=True)

        constraint.createObject("MechanicalObject", name="dofs",
                                template="Vec3d", position="@TransformEngine.output_position",
                                showObject=True, showObjectScale=10.0)

        constraint.createObject('RigidMapping', name="mapping", input=self.node.ServoMotor.ServoWheel.dofs, output="@./")

        return constraint


def createScene(rootNode):
    from splib.animation import animate
    from stlib.scene import Scene
    scene = Scene(rootNode)
    scene.createObject("EulerImplicitSolver")
    scene.createObject("SparseLDLSolver")
    scene.VisualStyle.displayFlags = "showBehavior"

    arm1 = ActuatedArm(scene, name="arm1", translation=[-2.0, 0.0, 0.0])
    arm1.createObject("FixedConstraint")

    def myanimate(target, factor):
        target.angle = factor

    animate(myanimate, {"target" : arm1.ServoMotor},
                        duration=0.5, mode="pingpong")
