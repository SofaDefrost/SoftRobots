import Sofa
from stlib.solver import DefaultSolver
from stlib.numerics import *
from stlib.scene import Node

class KinematicMotorController(Sofa.PythonScriptController):
    def __init__(self, node, parentframe, target):
        self.parentframe = parentframe
        self.node = node
        self.target = target
        self.addNewData("angle", "Properties", "The angular position of the motor (in radians)","f", 0.0)
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

def ServoMotor(parentNode, color="white"):
    servoMotor = Node(parentNode, 'ServoMotor')

    parentFrame = servoMotor.createObject("MechanicalObject", size=1,
                                          template='Rigid', showObject=True, showObjectScale=0.5)
    servoMotor.createObject('RigidRigidMapping')

    servoWheel = Node(servoMotor, "ServoWheel")
    meca = servoWheel.createObject("MechanicalObject", size=1, template='Rigid',
                                   showObject=True, showObjectScale=0.5)

    controller = KinematicMotorController(servoMotor, parentFrame, meca)

    visu = Node(servoMotor, "Visual")
    visu.createObject('MeshSTLLoader', name="Loader", filename='data/mesh/SG90_servo_with_base.stl')
    visu.createObject('OglModel', position='@Loader.position', triangles='@Loader.triangles', color=color)
    visu.createObject('RigidMapping')

    return servoMotor

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

def ActuatedArm(parentNode, name="ActuatedArm",
                            translation=[0.0,0.0,0.0], eulerRotation=[0.0,0.0,0.0]):
    actuatedArm = Node(parentNode, name)
    DefaultSolver(actuatedArm)
    r=Transform(translation, eulerRotation=eulerRotation)
    actuatedArm.createObject("MechanicalObject", size=1, position=r.toSofaRepr(), template='Rigid', showObject=True, showObjectScale=0.5)

    ServoMotor(actuatedArm)
    ServoArm(actuatedArm)

    return actuatedArm


def createScene(rootNode):
    from stlib.scene import MainHeader
    from stlib.solver import DefaultSolver
    from stlib.animation import animate, AnimationManager
    from stlib.animation.easing import LinearRamp
    from stlib.algorithms import get
    MainHeader(rootNode)
    DefaultSolver(rootNode)
    AnimationManager(rootNode)

    ### Test a assembly that also implements a KinematicMotorController
    ## The angle of the KinematicMotorController is dynamically changed using a
    ## animation function
    a = ActuatedArm(rootNode, translation=[2,0,0])
    def myAnimation(motorctrl, factor):
        motorctrl.angle = LinearRamp(-3.14/2, 3.14/2, factor)

    animate(myAnimation, {"motorctrl" : get(a, "ServoMotor/KinematicMotorController") }, duration=1.0, mode="pingpong")
