# -*- coding: utf-8 -*-
""" ActuatedArm for the tripod robot.

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    Available prefab:
        - ActuatedArm
        - ServoArm
"""
from splib3.numerics import vec3
from splib3.objectmodel import SofaPrefab
from stlib3.visuals import VisualModel
from stlib3.components import addOrientedBoxRoi

from s90servo import ServoMotor


@SofaPrefab
class ServoArm(object):
    def __init__(self, parent, mappingInput, name="ServoArm", indexInput=0):
        """ServoArm is a reusable sofa model of a servo arm for the S90 servo motor

           Parameters:
                parent:        node where the ServoArm will be attached
                mappingInput:  the rigid mechanical object that will control the orientation of the servo arm
                indexInput: (int) index of the rigid the ServoArm should be mapped to
        """
        self.node = parent.addChild(name)
        self.node.addObject("MechanicalObject",
                               name="dofs",
                               size=1,
                               template="Rigid3",
                               showObject=True,
                               showObjectScale=15)

        self.node.addObject('RigidRigidMapping',
                               name="mapping", input=mappingInput, index=indexInput)

        visual = VisualModel(self.node, '../../data/mesh/SG90_servoarm.stl', translation=[0., 0., 0.], color=[1., 1., 1., 0.75])
        visual.model.writeZTransparent = True
        visual.addObject('RigidMapping', name="mapping")


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

        self.node = parent.addChild(name)

        self.servomotor = ServoMotor(self.node, translation=translation, rotation=eulerRotation)
        ServoArm(self.node, self.servomotor.ServoWheel.dofs)

        if attachingTo is not None:
            constraint = self.addConstraint(attachingTo.dofs.getData("rest_position"), translation, eulerRotation)
            attachingTo.addObject('RestShapeSpringsForceField', name="rssff"+name,
                                     points=constraint.BoxROI.getData("indices"),
                                     external_rest_shape=constraint.dofs,
                                     stiffness='1e12')

    def addConstraint(self, position, translation, eulerRotation):
        constraint = self.node.addChild("Constraint")
        o = addOrientedBoxRoi(constraint, position=position,
                              translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                              eulerRotation=eulerRotation, scale=[45, 15, 30])
        o.drawSize = 1
        o.drawBoxes = False

        constraint.addObject("TransformEngine", input_position="@BoxROI.pointsInROI",
                                translation=translation, rotation=eulerRotation, inverse=True)

        constraint.addObject("MechanicalObject", name="dofs",
                                template="Vec3", position="@TransformEngine.output_position",
                                showObject=True, showObjectScale=5.0)

        constraint.addObject('RigidMapping', name="mapping", input=self.node.ServoMotor.ServoWheel.dofs, output="@./")

        return constraint


def createScene(rootNode):
    from splib.animation import animate
    from stlib.scene import Scene
    import math

    scene = Scene(rootNode)
    scene.VisualStyle.displayFlags = "showBehavior"
    rootNode.dt = 0.003
    rootNode.gravity = [0., -9810., 0.]

    simulation = rootNode.addChild("Simulation")
    simulation.addObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
    simulation.addObject("CGLinearSolver", name="precond")

    arm = ActuatedArm(simulation, name="ActuatedArm", translation=[0.0, 0.0, 0.0])

    def myanimate(target, factor):
        target.angle = math.cos(factor * 2 * math.pi)
        target.setX(math.cos(factor * 2 * math.pi))

    animate(myanimate, {"target": arm.servomotor}, duration=5, mode="loop")
