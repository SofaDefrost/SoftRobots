import os
import Sofa
from splib3.objectmodel import SofaObject, SofaPrefab
from splib3.animation import animate
from stlib3.scene import Scene
dirPath = os.path.dirname(os.path.abspath(__file__))+'/../'


def ServoBody(parent, position=[0., 0., 0., 0., 0., 0., 1.], showServo=True):

    servobody = parent.addChild('ServoBody')
    servobody.addObject('MechanicalObject', template='Rigid3', name='dofs', position=position)
    servobody.addObject('MeshTopology')

    if showServo:
        visual = servobody.addChild('VisualModel')
        visual.addObject('MeshSTLLoader', name='loader', filename=dirPath+'data/mesh/SG90_servomotor.stl')
        visual.addObject('MeshTopology', src='@loader')
        visual.addObject('OglModel', color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        visual.addObject('RigidMapping', index=0)

    return servobody


def ServoWheel(parent, showWheel=True):

    servowheel = parent.addChild('ServoWheel')
    servowheel.addObject('MechanicalObject', template='Rigid3', name='dofs', position=[[0., 0., 0., 0., 0., 0., 1.]],
                            showObject=showWheel, showObjectScale=10)
    servowheel.addObject('MeshTopology')

    return servowheel


class ServoMotor(Sofa.Prefab):
    '''A S90 servo motor

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

    def addScene(root):
        ...
        servo = ServoMotor(root)

        ## Direct access to the components
        servo.angle.value = 1.0
    '''
    properties = [
        {'name':'name',                'type':'string', 'help':'Node name',                   'default':'ServoMotor'},
        {'name':'rotation',            'type':'Vec3d',  'help':'Rotation',                    'default':[0.0, 0.0, 0.0]},
        {'name':'translation',         'type':'Vec3d',  'help':'Translation',                 'default':[0.0, 0.0, 0.0]},
        {'name':'scale3d',               'type':'Vec3d',  'help':'Scale 3d',                    'default':[1.0, 1.0, 1.0]}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        showServo=True
        showWheel=False

        # The inputs
        self.addData(name='minAngle', group='S90Properties', help='min angle of rotation (in radians)', type='float', value=-100)
        self.addData(name='maxAngle', group='S90Properties', help='max angle of rotation (in radians)', type='float', value=100)
        self.addData(name='angleIn', group='S90Properties', help='angle of rotation (in radians)', type='float', value=0)

        # Two positions (rigid): first one for the servo body, second for the servo wheel
        baseFrame = self.addChild('BaseFrame')
        baseFrame.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.]],
                               translation=list(self.translation.value),rotation=list(self.rotation.value),scale3d=list(self.scale3d.value))

        # Angle of the wheel
        angle = self.addChild('Angle')
        angle.addObject('MechanicalObject', name='dofs', template='Vec1d', position=self.getData('angleIn').getLinkPath())
        # This component is used to constrain the angle to lie between a maximum and minimum value,
        # corresponding to the limit of the real servomotor
        angle.addObject('ArticulatedHierarchyContainer')
        angle.addObject('ArticulatedSystemMapping', input1=angle.dofs.getLinkPath(), output=baseFrame.dofs.getLinkPath())
        angle.addObject('StopperConstraint', name='AngleLimits', index=0, min=self.getData('minAngle').getLinkPath(), max=self.getData('maxAngle').getLinkPath())
        # angle.addObject('UncoupledConstraintCorrection')

        articulationCenter = angle.addChild('ArticulationCenter')
        articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.], posOnChild=[0., 0., 0.])
        articulation = articulationCenter.addChild('Articulations')
        articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=0)

        # ServoBody and ServoWheel objects with visual
        servowheel = ServoWheel(self, showWheel=showWheel)
        servowheel.addObject('RigidRigidMapping', input=self.BaseFrame.dofs.getLinkPath(), output=servowheel.dofs.getLinkPath(), index=1)
        servobody = ServoBody(self, showServo=showServo)
        servobody.addObject('RigidRigidMapping', input=self.BaseFrame.dofs.getLinkPath(), output=servobody.dofs.getLinkPath(), index=0)

        # The output
        self.addData(name='angleOut', group='S90Properties', help='angle of rotation (in degree)', type='float', value=angle.dofs.getData('position').getLinkPath())

        self.BaseFrame.init()
        self.BaseFrame.dofs.rotation = [0., 0., 0.]
        self.BaseFrame.dofs.translation = [0., 0., 0.]


def createScene(rootNode):

    import math

    # def animation(target, factor):
    #     target.angleIn = math.cos(factor * 2 * math.pi)

    # Scene(rootNode)

    # rootNode.dt = 0.003
    # rootNode.gravity = [0., -9810., 0.]
    # rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels')

    # # Use these components on top of the scene to solve the constraint 'StopperConstraint'.
    # rootNode.addObject('FreeMotionAnimationLoop')
    # rootNode.addObject('GenericConstraintSolver', maxIterations=1e3, tolerance=1e-5)

    # simulation = rootNode.addChild('Simulation')
    # simulation.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    # simulation.addObject('CGLinearSolver', name='precond')

    # simulation.addChild(ServoMotor(name="ServoMotor"))
    # animate(animation, {'target': simulation.ServoMotor}, duration=5., mode='loop')

    servo = rootNode.addChild(ServoMotor(name="ServoMotor"))
    print(servo.BaseFrame.dofs)
    return rootNode
