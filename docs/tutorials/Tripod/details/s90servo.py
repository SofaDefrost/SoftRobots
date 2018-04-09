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
from stlib.solver import DefaultSolver
from stlib.numerics import *
from stlib.scene import Node, get

def ServoMotor(parentNode, translation=[0.0,0.0,0.0], rotation=[0.0,0.0,0.0], scale=[1.0,1.0,1.0],
               inFrame=False, color="white"):
    servoMotor = Node(parentNode, 'ServoMotor')
    servoMotor.addNewData("angle",  "ServoMotor Properties", "The angular position of the motor (in radians)","d", 0.0)

    parentFrame = servoMotor.createObject("MechanicalObject", size=1, 
                                          translation=translation, rotation=rotation, scale=scale,
                                          template='Rigid', showObject=True, showObjectScale=0.5)

    if inFrame:
        servoMotor.createObject('RigidRigidMapping')

    servoWheel = Node(servoMotor, "ServoWheel")
    meca = servoWheel.createObject("MechanicalObject", size=1, template='Rigid',
                                   showObject=True, showObjectScale=0.5)

    controller = KinematicMotorController(servoMotor, parentFrame,
                                          meca, angleValue=get(servoMotor, "angle").getLinkPath())

    visu = Node(servoMotor, "Visual")
    visu.createObject('MeshSTLLoader', name="Loader", filename='data/mesh/SG90_servo_with_base.stl')
    visu.createObject('OglModel', position='@Loader.position', triangles='@Loader.triangles', color=color)
    visu.createObject('RigidMapping')
    
    return servoMotor

class KinematicMotorController(Sofa.PythonScriptController):
    """
        This controller is in charge of transforming the angular 'positional' control of the
        of the ServoWheel.
    """
    def __init__(self, node, parentframe, target, angleValue):
        self.parentframe = parentframe
        self.node = node
        self.target = target
        self.addNewData("angle", "Properties", "The angular position of the motor (in radians)","d", angleValue)
        self.name = "KinematicMotorController"

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
    from stlib.scene import MainHeader
    from stlib.animation import animate
    from stlib.animation.easing import LinearRamp
    from stlib.algorithms import get
    MainHeader(rootNode)
    s=DefaultSolver(rootNode)

    ### Test a assembly that also implements a KinematicMotorController
    ## The angle of the KinematicMotorController is dynamically changed using a
    ## animation function
    servoMotor = ServoMotor(rootNode, translation=[2,0,0])
    def myAnimation(motorctrl, factor):
        motorctrl.angle = LinearRamp(-3.14/2, 3.14/2, factor)

    animate(myAnimation, {"motorctrl" : servoMotor }, duration=1.0, mode="pingpong")
