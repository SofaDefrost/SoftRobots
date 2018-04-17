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
from splib.numerics import *
from splib.objectmodel import *

from stlib.scene import Node, get
from stlib.visuals import VisualModel
from stlib.physics.deformable import ElasticMaterialObject

from s90servo import ServoMotor

@SofaPrefab
class ServoArm(SofaObject):
    def __init__(self, parent, mappingInput, name="ServoArm", addVisualModel=True):
        """ServoArm is a reusable sofa model of a servo arm for the S90 servo motor

           Parameters:
                parent:        node where the ServoArm will be attached
                mappingInput:  the rigid mechanical object that will control the orientation of the servo arm
                addVisualTrue: add the dedicated visual model. 

           Structure:
           Node : {
                name : "ActuatedArm"
                MechanicalObject     // Rigid position of the motor
                ServoMotor           // The s90 servo motor with it actuated wheel
                ServoArm             // The actuation arm connected to ServoMotor/ServoWheel
            }
        """

        self.node  = Node(parent, name)
        self.dofs = self.node.createObject("MechanicalObject", 
                                           name="dofs", 
                                           size=1, 
                                           template='Rigid', 
                                           showObject=True, 
                                           showObjectScale=0.5)
        
        self.mapping = self.node.createObject('RigidRigidMapping', name="mapping", input=mappingInput)

        if addVisualModel:
            self.addVisualModel() 

    def addVisualModel(self):
        self.visualmodel = VisualModel(self.node, 'data/mesh/servo_arm_assembly.stl')
        self.visualmodel.node.createObject('RigidMapping', name="mapping")
        return self.visualmodel


@SofaPrefab
class ActuatedArm(SofaObject):
    def __init__(self, parent, name="ActuatedArm", translation=[0.0,0.0,0.0], eulerRotation=[0.0,0.0,0.0]):
        """ActuatedArm is a reusable sofa model of a S90 servo motor and the tripod actuation arm.
           Parameters:
             - translation the position in space of the structure
             - eulerRotation the orientation of the structure

           Structure:
           Node : {
                name : "ActuatedArm"
                MechanicalObject     // Rigid position of the motor
                ServoMotor           // The s90 servo motor with it actuated wheel
                ServoArm             // The actuation arm connected to ServoMotor.ServoWheel
            }
        """
        self.node = Node(parent, name)
        r=Transform(translation, eulerRotation=eulerRotation)
        self.node.createObject("MechanicalObject", name="dofs", size=1, position=r.toSofaRepr(), 
                               template='Rigid', showObject=True, showObjectScale=0.5)

        self.servomotor = ServoMotor(self.node)
        self.servomotor.node.createObject("RigidRigidMapping", name="mapping")
        self.servoarm = ServoArm(self.node, self.servomotor.servowheel.dofs)

@SofaPrefab
class ElasticBody(SofaObject):
    def __init__(self, parent, eulerRotation=[0.0,0.0,0.0], youngModulus=600, totalMass=0.4, name="ElasticBody"):         
        self.node = Node(parent, name)
        self.createPrefab(self, eulerRotation=eulerRotation)          
                   
    def createPrefab(self, node, eulerRotation):
        return ElasticMaterialObject.createPrefab(node, rotation=eulerRotation, volumeMeshFileName="data/mesh/tripod1.gidmsh")
         
    def addVisualModel(self):
        ElasticMaterialObject.addVisualModel(self)
        
    def addCollisionModel(self, filename=None):
        if filename != None:
            ElasticMaterialObject.addCollisionModel(self, filename)
        else:
            self.collisionmodel = SofaObject(self.node, "CollisionModel")
            self.collisionmodel.container = self.collisionmodel.node.createObject("TriangleSetTopologyContainer", name="container")
            self.collisionmodel.node.createObject("TriangleSetTopologyModifier", name="modifier")    
            self.collisionmodel.node.createObject("TriangleSetTopologyAlgorithms", name="topologyalgorithms")    
            self.collisionmodel.node.createObject("TriangleSetGeometryAlgorithms", name="topologicalmapping")    
            self.collisionmodel.node.createObject("Tetra2TriangleTopologicalMapping", name="topologicalmapping", 
                                                  input=self.container.getLinkPath(), output=self.collisionmodel.container)
            self.collisionmodel.node.createObject("TriangleModel")
        
            
def createScene(rootNode):
    from stlib.scene import Scene
    from stlib.solver import DefaultSolver
    from splib.animation import animate, AnimationManager
    from splib.animation.easing import LinearRamp
    from splib.scenegraph import get
    scene = Scene(rootNode)
    scene.visualstyle.displayFlags = "showForceFields showBehavior"
    scene.addSolver()

    ### Test a assembly that also implements a KinematicMotorController
    ## The angle of the KinematicMotorController is dynamically changed using a
    ## animation function
    aa = ActuatedArm(rootNode, translation=[3,7,0])
    eb = ElasticBody(rootNode, eulerRotation=[90.0,0.0,0.0])
    eb.addCollisionModel()
    def myAnimation(actuatedarm, factor):
        actuatedarm.servomotor.angle.value = LinearRamp(-3.14/2, 3.14/2, factor)

    animate(myAnimation, {"actuatedarm" : aa}, duration=1.0, mode="pingpong")
    
