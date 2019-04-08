# -*- coding: utf-8 -*-
""" ActuatedArm for the tripod robot.

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    Available prefab:
        - ActuatedArm
        - ServoArm
"""
from splib.numerics import vec3, RigidDof
from splib.objectmodel import SofaPrefab
from stlib.visuals import VisualModel
from stlib.components import addOrientedBoxRoi

from s90servo import ServoMotor


@SofaPrefab
class ServoArm(object):
    def __init__(self, parent, mappingInput, name="ServoArm", indexInput=1):
        """ServoArm is a reusable sofa model of a servo arm for the S90 servo motor

           Parameters:
                parent:        node where the ServoArm will be attached
                mappingInput:  the rigid mechanical object that will control the orientation of the servo arm
                indexInput: (int) index of the rigid the ServoArm should be mapped to
        """
        self.node = parent.createChild(name)
        self.node.createObject("MechanicalObject",
                               name="dofs",
                               size=1,
                               template="Rigid3",
                               showObject=True,
                               showObjectScale=15)

        self.node.createObject('RigidRigidMapping',
                               name="mapping", input=mappingInput, index=indexInput)

        visual = VisualModel(self.node, 'data/mesh/SG90_servoarm.stl', translation=[0., 0., 0.], color=[1., 1., 1., 0.75])
        visual.model.writeZTransparent = True
        visual.createObject('RigidMapping', name="mapping")


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

        self.servomotor = ServoMotor(self.node, translation=translation, rotation=eulerRotation)
        ServoArm(self.node, self.servomotor.BaseFrame.ArticulatedJoint.WheelFrame.slavedofs)

        if attachingTo is not None:
            constraint = self.addConstraint(self.servomotor.BaseFrame.ArticulatedJoint.WheelFrame, 
                                            attachingTo.dofs.getData("rest_position"), translation, eulerRotation)
            attachingTo.createObject('RestShapeSpringsForceField', name="rssff"+name,
                                     points=constraint.BoxROI.getData("indices"),
                                     external_rest_shape=constraint.dofs,
                                     stiffness='1e12')

        self.node.init()

    def addHandle(self, translation,eulerRotation=None):
        constraint = self.node.createChild("Handler")
        o = addOrientedBoxRoi(constraint, 
                              name="boxroi",
                              position=[0.0,0.0,0.0],
                              translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                              eulerRotation=eulerRotation, scale=[45, 15, 30])
        
        m = constraint.createObject("MechanicalObject", name="slavedofs", template="Rigid3", position=[0.0,25.0,0.0,0,0,0,1])
        constraint.createObject("RigidRigidMapping", 
                               input=self.servomotor.BaseFrame.ArticulatedJoint.WheelFrame.slavedofs.getLinkPath(), index=1, 
                               output=m.getLinkPath())
        
        m.showObject=True
        m.showObjectScale=10.0
        m.drawMode = 4
        o.drawSize = 1
        o.drawBoxes = False
        
        constraint.init()
        return constraint    

    def addConstraint(self, node, position, translation, eulerRotation):
        constraint = node.createChild("Constraint")
        o = addOrientedBoxRoi(constraint, position=position,
                              translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                              eulerRotation=eulerRotation, scale=[45, 15, 30])
        o.drawSize = 1
        o.drawBoxes = False
       
        constraint.createObject("MechanicalObject", name="dofs",
                                template="Vec3d", position="@BoxROI.pointsInROI",
                                showObject=True, showObjectScale=5.0)

        constraint.createObject('RigidMapping', name="mapping", input=self.node.ServoMotor.BaseFrame.ArticulatedJoint.WheelFrame, output="@./", index=1,
                                globalToLocalCoords=True)

        return constraint


def createScene(rootNode):
    from splib.animation import animate
    from stlib.scene import Scene
    from splib.objectmodel import setData
    import math

    scene = Scene(rootNode)
    scene.VisualStyle.displayFlags = "showBehavior"
    rootNode.dt = 0.003
    rootNode.gravity = [0., -9810., 0.]

    simulation = rootNode.createChild("Simulation")
    simulation.createObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
    simulation.createObject("CGLinearSolver", name="precond")

    arm = ActuatedArm(simulation, name="ActuatedArm", translation=[20.0, 40.0, 0.0])
    arm.addHandle(translation=[0,0,0])
    
    setData(arm.ServoMotor.BaseFrame.dofs,  showObject=True, showObjectScale=10)
    setData(arm.ServoMotor.BaseFrame.ArticulatedJoint.WheelFrame.slavedofs,  showObject=True, showObjectScale=10)
    setData(arm.ServoArm.dofs,  showObject=True, showObjectScale=10)
    
    def myanimate(target, factor):
        target.angle = math.cos(factor * 2 * math.pi)
        target.setX(math.cos(factor * 2 * math.pi))

    animate(myanimate, {"target": arm.servomotor}, duration=5, mode="loop")
