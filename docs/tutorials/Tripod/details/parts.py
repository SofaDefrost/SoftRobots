""" Sofa prefab for a S90 servo actuators with a controller

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    List of available parts:
        - ActuatedArm (assemblage of a ServoMotor and a ServoArm)
        - ServoMotor
        - ServoArm

    Other public components:
        - KinematicMotorController

"""
import Sofa
from stlib.numerics import *
from stlib.scene import Node, get
from stlib.physics.deformable import ElasticMaterialObject
from s90servo import ServoMotor, KinematicMotorController

def ActuatedArm(parentNode, name="ActuatedArm",
                            translation=[0.0,0.0,0.0], eulerRotation=[0.0,0.0,0.0]):
    """ActuatedArm is a reusable sofa model of a S90 servo motor and the tripod actuation arm.
       Parameters:
           - translation the position in space of the structure
           - eulerRotation the orientation of the structure

       Structure:
       Node : {
            name : "ActuatedArm"
            MechanicalObject; // Rigid position of the motor
            ServoMotor        // The s90 servo motor with it actuated wheel
            ServoArm          // The actuation arm connected to ServoMotor.ServoWheel
       }

    """
    actuatedArm = Node(parentNode, name)
    r=Transform(translation, eulerRotation=eulerRotation)
    actuatedArm.createObject("MechanicalObject", size=1, position=r.toSofaRepr(),
    template='Rigid', showObject=True, showObjectScale=0.5)

    ServoMotor(actuatedArm, inFrame=True)
    ServoArm(actuatedArm)

    return actuatedArm


def ElasticBody(parentNode, rotation=[0.0,0.0,0.0], youngModulus=600, totalMass=0.4, name="ElasticBody"):
    eb = parentNode.createChild(name)

    em = ElasticMaterialObject(eb, 
                         volumeMeshFileName="data/mesh/tripod1.gidmsh",
                         totalMass=totalMass, poissonRatio=0.45, youngModulus=youngModulus,
                         rotation=rotation, solver=True,withConstrain=False)

    return eb

def addCollisionTo(eb):
    em = get(eb, "ElasticMaterialObject")   
    
    ec=em.createChild("Collision")
    c = ec.createObject("TriangleSetTopologyContainer", name="TriangleSetTopologyContainer")
    m = ec.createObject("TriangleSetTopologyModifier", name="TriangleSetTopologyModifier")    
    ec.createObject("TriangleSetTopologyAlgorithms", name="TriangleSetTopologyModifierX")    
    ec.createObject("TriangleSetGeometryAlgorithms", name="TriangleSetTopologyModifierA")    
    em.createObject("LinearSolverConstraintCorrection")
    
    ec.createObject("Tetra2TriangleTopologicalMapping", 
                    input=get(em, "TetrahedronSetTopologyContainer").getLinkPath(),    output=c.getLinkPath())
    ec.createObject("TriangleModel")
    #ec.createObject("PointModel")
    #ec.createObject("IdentityMapping", input=get(em, "MechanicalObject").getLinkPath())
    return eb

def ServoArm(parentNode, color="white"):
    servoArm = Node(parentNode, 'ServoArm')

    meca = servoArm.createObject("MechanicalObject", size=1,
                                 template='Rigid', showObject=True, showObjectScale=0.5)
    servoArm.createObject('RigidRigidMapping', input="@../ServoMotor/ServoWheel")

    visual = servoArm.createChild("Visual")
    visual.createObject('MeshSTLLoader', name='MeshLoader',
                        filename='data/mesh/servo_arm_assembly.stl')
    visual.createObject('OglModel', name='OglModel',
                        position='@MeshLoader.position',
                        triangles='@MeshLoader.triangles', color=color)
    visual.createObject('RigidMapping')

    return servoArm

def createScene(rootNode):
    from stlib.scene import MainHeader
    from stlib.solver import DefaultSolver
    from stlib.animation import animate, AnimationManager
    from stlib.animation.easing import LinearRamp
    from stlib.algorithms import get
    MainHeader(rootNode)
    s=DefaultSolver(rootNode)
    AnimationManager(rootNode)

    ### Test a assembly that also implements a KinematicMotorController
    ## The angle of the KinematicMotorController is dynamically changed using a
    ## animation function
    a = ActuatedArm(rootNode, translation=[2,0,0])
    def myAnimation(motorctrl, factor):
        motorctrl.angle = LinearRamp(-3.14/2, 3.14/2, factor)

    animate(myAnimation, {"motorctrl" : get(a, "ServoMotor/KinematicMotorController") }, duration=1.0, mode="pingpong")
