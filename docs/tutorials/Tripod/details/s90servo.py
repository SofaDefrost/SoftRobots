""" Sofa prefab for a S90 servo actuators with a controller

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    List of available parts:
        - ServoMotor

    Other public components:
        - KinematicMotorController
        - ServoWheel 
"""
import Sofa
from splib.numerics import *
from splib.objectmodel import *  

from stlib.visuals import VisualModel 
from stlib.solver import DefaultSolver
from stlib.scene import Scene, Node, get

@SofaPrefab
class ServoMotor(SofaObject):
    """A S90 servo motor

    Use this prefab to get a S90 servo motor including:
        - a visual model
        - a simple mechanical model made of two rigids one for the the servo motor the other
          for the servo wheel.

    ServoMotor Properties:
        - angle     use this to specify the angle of rotation of the servo motor
    """
    def __init__(self, parent, 
                 translation=[0.0,0.0,0.0], rotation=[0.0,0.0,0.0], scale=[1.0,1.0,1.0], doAddVisualModel=True):
        self.node = Node(parent, "ServoMotor")
        self.node.addNewData("angle",  
                             "ServoMotor Properties", 
                             "The angular position of the motor (in radians)","d", 0.0)
        self.angle=self.node.findData("angle")
        
        self.dofs = self.node.createObject("MechanicalObject", size=1, 
                                          name="dofs",
                                          translation=translation, rotation=rotation, scale=scale,
                                          template='Rigid', showObject=True, showObjectScale=0.5)
        
        self.servowheel = ServoWheel(self.node)
        self.controller = KinematicMotorController(self.node, self.dofs, self.servowheel.dofs, 
                                                   angleValue=self.angle.getLinkPath())
                                                   
        if doAddVisualModel:
            self.addVisualModel()        

    def addVisualModel(self):
        self.visualmodel = VisualModel(self.node, 'data/mesh/SG90_servo_with_base.stl')
        self.visualmodel.node.createObject('RigidMapping', name = "mapping")

class ServoWheel(SofaObject):
    def __init__(self, parentNode):
        self.node = Node(parentNode, "ServoWheel")
        self.dofs = self.node.createObject("MechanicalObject", size=1, template='Rigid',
                                           showObject=True, showObjectScale=0.5, name="dofs")

class KinematicMotorController(Sofa.PythonScriptController):
    """
        This controller is in charge of transforming the angular 'positional' control of the
        of the ServoWheel.
    """
    def __init__(self, node, parentframe, target, angleValue):
        self.name = "controller"
        self.parentframe = parentframe
        self.node = node
        self.target = target
        self.addNewData("angle", "Properties", "The angular position of the motor (in radians)","d", angleValue)

    def applyAngleToServoWheel(self, angle):
        pq = self.parentframe.position[0]
        local = Transform(pq[0:3], pq[3:7])
        f = local.forward
        self.target.findData("position").value = pq[0:3] + list(Quaternion.prod(axisToQuat(f, angle), pq[3:7]))

    def bwdInitGraph(self, root):
        self.applyAngleToServoWheel(self.angle)

    def onBeginAnimationStep(self, dt):
        self.applyAngleToServoWheel(self.angle)


def createScene(rootNode):
    from splib.animation import animate
    from splib.animation.easing import LinearRamp      
    from splib.scenegraph import get
        
    from stlib.scene import MainHeader
    Scene(rootNode)
    s=DefaultSolver(rootNode)

    ### Test a assembly that also implements a KinematicMotorController
    ## The angle of the KinematicMotorController is dynamically changed using a
    ## animation function
    servomotor = ServoMotor(rootNode, translation=[2,0,0])
    def myAnimation(motorctrl, factor):
        motorctrl.angle = LinearRamp(-3.14/2, 3.14/2, factor)

    animate(myAnimation, {"motorctrl" : servomotor.node }, duration=1.0, mode="pingpong")
