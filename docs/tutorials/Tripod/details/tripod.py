""" Sofa model for the "tripod' robot

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    List of available parts:
        - Tripod
        - ActuatedArmWithConstraint (add to a s90 ActuatedArm a set of constraints to attach a deformable object )

    Example of use:
        def createScene(rootNode):
            from stlib.scene import MainHeader
            r = MainHeader(rootNode, plugins=["SoftRobots"])
            r.getObject("VisualStyle").displayFlags="showForceFields"

            tripod = Tripod(rootNode)

            return rootNode
"""
from stlib.scene import Node
from stlib.physics.deformable import ElasticMaterialObject
from stlib.components import OrientedBoxRoi
from stlib.algorithms import get
from stlib.numerics import *
from math import sin,cos
from parts import ActuatedArm

def ActuatedArmWithConstraint(parentNode, name="ActuatedArm", position=[], translation=[0,0,0], eulerRotation=[0,0,0]):

    arm = ActuatedArm(parentNode, name=name,
                      translation=translation,
                      eulerRotation=eulerRotation)

    constraint = Node(arm, "Constraint")
    OrientedBoxRoi(constraint, position=position,
                     translation=vadd(translation, [0.0,1.0,0.0]),
                     eulerRotation=eulerRotation, scale=[2.5,1,0.7])

    constraint.createObject("TransformEngine", input_position="@BoxROI.pointsInROI",
                            translation=translation, rotation=eulerRotation, inverse=True )

    constraint.createObject("MechanicalObject", template="Vec3d", position="@TransformEngine.output_position",
                            showObject=True, showObjectScale=10.0)

    constraint.createObject('RigidMapping', input="@../ServoMotor/ServoWheel/MechanicalObject", output="@./")

    return arm

def Tripod(parentNode):
    tripod = Node(parentNode, 'Tripod')

    em=ElasticMaterialObject(tripod,
                         volumeMeshFileName="data/mesh/tripod1.gidmsh",
                         totalMass=0.4, poissonRatio=0.45, youngModulus=600, rotation=[90,0,0])
    dist = 4.6
    numstep = 3
    for i in range(0,numstep):
        name = "ActuatedArm"+str(i)
        fi = float(i)
        fnumstep = float(numstep)
        angle = fi*360/fnumstep
        angle2 = fi*360/fnumstep+180
        eulerRotation = [0,angle,0]
        translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]

        c = ActuatedArmWithConstraint(tripod, name=name, position="@../../ElasticMaterialObject/MechanicalObject.rest_position",
                                      translation=translation, eulerRotation=eulerRotation)

        em.createObject('RestShapeSpringsForceField',
                    points=get(c, 'Constraint/BoxROI.indices').getLinkPath(),
                    external_rest_shape=get(c, 'Constraint/MechanicalObject').getLinkPath())

    return tripod

#Units: cm and kg
def createScene(rootNode):
    from stlib.scene import MainHeader
    r = MainHeader(rootNode, plugins=["SoftRobots"])
    r.getObject("VisualStyle").displayFlags="showForceFields"

    tripod = Tripod(rootNode)

    return rootNode
