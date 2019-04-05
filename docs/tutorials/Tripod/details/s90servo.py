import os
from splib.objectmodel import SofaObject, SofaPrefab
from splib.animation import animate
from stlib.scene import Scene
dirPath = os.path.dirname(os.path.abspath(__file__))+'/'


def ServoBody(parent, position=[0., 0., 0., 0., 0., 0., 1.], showServo=True):

    servobody = parent.createChild("ServoBody")
    servobody.createObject("MechanicalObject", template="Rigid3d", name="dofs", position=position)
    servobody.createObject("MeshTopology")

    if showServo:
        visual = servobody.createChild("VisualModel")
        visual.createObject("MeshSTLLoader", name="loader", filename=dirPath+"data/mesh/SG90_servomotor.stl")
        visual.createObject("MeshTopology", src="@loader")
        visual.createObject("OglModel", color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        visual.createObject("RigidMapping", index=0)

    return servobody


def ServoWheel(parent, showWheel=True):

    servowheel = parent.createChild("ServoWheel")
    servowheel.createObject("MechanicalObject", template="Rigid3d", name="dofs", position=[[0., 0., 0., 0., 0., 0., 1.]],
                            showObject=showWheel, showObjectScale=10)
    servowheel.createObject("MeshTopology")

    return servowheel


@SofaPrefab
class ServoMotor(SofaObject):
    """A S90 servo motor

    This prefab is implementing a S90 servo motor.
    https://servodatabase.com/servo/towerpro/sg90

    The prefab ServoMotor is composed of:
    - a visual model
    - a mechanical model composed two rigids. One rigid is for the motor body
      while the other is implementing the servo rotating wheel.

    The prefab has the following parameters:
    - translation           to change default location of the servo (default [0.0,0.0,0.0])
    - rotation              to change default rotation of the servo (default [0.0,0.0,0.0,1])
    - scale                 to change default scale of the servo (default 1)
    - showServo             to control wether a visual model of the motor is added (default True)
    - showWheel             to control wether the rotation axis of the motor is displayed (default False)

    The prefab have the following property:
    - angle         use this to specify the angle of rotation of the servo motor
    - angleLimits   use this to set a min and max value for the servo angle rotation
    - position      use this to specify the position of the servo motor

    Example of use in a Sofa scene:

    def createScene(root):
        ...
        servo = ServoMotor(root)

        ## Direct access to the components
        servo.angle.value = 1.0
    """

    def __init__(self, parent,
                 translation=[0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0], scale=[1.0, 1.0, 1.0], showServo=True, showWheel=False):

        self.node = parent.createChild("ServoMotor")

        # Two positions (rigid): first one for the servo body, second for the servo wheel
        position = self.node.createChild("Position")
        position.createObject("MechanicalObject", name="dofs", template="Rigid3", position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.]],
                              translation=translation, rotation=rotation, scale3d=scale)

        # Angle of the wheel
        angle = self.node.createChild("Angle")
        angle.createObject("MechanicalObject", name="dofs", template="Vec1d", position=0)
        # This component is used to constrain the angle to lie between a maximum and minimum value,
        # corresponding to the limit of the real servomotor
        angle.createObject("ArticulatedHierarchyContainer")
        angle.createObject("ArticulatedSystemMapping", input1=angle.dofs.getLinkPath(), output=position.dofs.getLinkPath())
        angle.createObject('StopperConstraint', name="AngleLimits", index=0)
        angle.createObject("UncoupledConstraintCorrection")

        articulationCenter = angle.createChild("ArticulationCenter")
        articulationCenter.createObject("ArticulationCenter", parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.], posOnChild=[0., 0., 0.])
        articulation = articulationCenter.createChild("Articulations")
        articulation.createObject("Articulation", translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=0)

        # ServoBody and ServoWheel objects with visual
        servobody = ServoBody(self.node, showServo=showServo)
        servobody.createObject("RigidRigidMapping", input=self.node.Position.dofs.getLinkPath(), output=servobody.dofs.getLinkPath(), index=0)
        servowheel = ServoWheel(self.node, showWheel=showWheel)
        servowheel.createObject("RigidRigidMapping", input=self.node.Position.dofs.getLinkPath(), output=servowheel.dofs.getLinkPath(), index=1)

    def setAngle(self, angle):
        self.node.Angle.dofs.position = angle

    def getAngle(self):
        return self.node.Angle.dofs.position[0][0]

    def setAngleLimits(self, angleLimits):
        self.node.Angle.AngleLimits.min = angleLimits[0]
        self.node.Angle.AngleLimits.max = angleLimits[1]

    angle = property(getAngle, setAngle)
    angleLimits = property(fset=setAngleLimits)

    def setX(self, x, index=0):
        position = self.Position.dofs.position
        position[index][0] = x
        self.Position.dofs.position = position


def createScene(rootNode):

    import math

    def animation(target, factor):
        target.angle = math.cos(factor * 2 * math.pi)
        target.setX(math.cos(factor * 2 * math.pi))

    Scene(rootNode)

    rootNode.dt = 0.003
    rootNode.gravity = [0., -9810., 0.]
    rootNode.createObject("VisualStyle", displayFlags="showBehaviorModels")

    # Use these components on top of the scene to solve the constraint "StopperConstraint".
    rootNode.createObject("FreeMotionAnimationLoop")
    rootNode.createObject("GenericConstraintSolver", maxIterations=1e3, tolerance=1e-5)

    simulation = rootNode.createChild("Simulation")
    simulation.createObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
    simulation.createObject("CGLinearSolver", name="precond")

    servomotor = ServoMotor(simulation, showWheel=True)
    animate(animation, {"target": servomotor}, duration=5., mode="loop")

    return rootNode
