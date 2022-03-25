# -*- coding: utf-8 -*-
"""
Step 7: Here we are showing how to setup a communication with the servomotors
"""
import Sofa
from tutorial import *
from splib3.animation import animate
from splib3.constants import Key
from math import floor, pi
from tripod import Tripod
from tripodcontroller import TripodController, setupanimation


class TripodControllerWithCom(TripodController):
    """This controller has three roles:
       - if the user presses up/left/right/down/plus/minus, the servomotor angle
         is changed.
       - if the user presses A, an animation is started to move the servomotor to the initial position
         of the real robot.
       - if thr user presses B start the communication with controller card, send
         servomotor commands
    """

    def __init__(self, *args, **kwargs):
        TripodController.__init__(self, *args, **kwargs)
        self.serialportctrl = kwargs['serialportctrl']

    def initTripod(self, key):
        if key == Key.A and self.serialportctrl.state == "init":
            self.serialportctrl.state = "no-comm"
            animate(setupanimation, {"actuators": self.actuators, "step": 35.0, "angularstep": -1.4965}, duration=0.2)

        # Inclusion of the keystroke to start data sending = establishing communication ('comm')
        if key == Key.B and self.serialportctrl.state == "no-comm":
            self.serialportctrl.state = "comm"


# Description of how the communication is handled
# CHANGE HERE the serialport that correspond to your computer
# def SerialPortBridgeGeneric(rootNode, serialport='/dev/cu.usbserial-1420'):
# def SerialPortBridgeGeneric(rootNode, serialport='COM3'):
def SerialPortBridgeGeneric(rootNode, serialport='/dev/ttyUSB0'):
    return rootNode.addObject('SerialPortBridgeGeneric', port=serialport, baudRate=115200, size=3, listening=True,
                              header=255)


# Data sending controller
class SerialPortController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "serialportcontroller"
        self.actuatedarms = kwargs['inputs']
        self.serialport = kwargs['serialport']
        self.serialport.packetOut = [150, 150, 150]
        self.state = "init"

    def onAnimateEndEvent(self, event):
        # Data sending if the robot is initializing or in the no-communication state
        if self.state == "init":
            return

        if self.state == "no-comm":
            return

        # Vector storing the simulated servomotors' angular position
        angles = []

        for arm in self.actuatedarms:
            # Conversion of the angle values from radians to degrees
            angleDegree = arm.servomotor.angleOut.value * 360 / (2.0 * pi)
            angleByte = int(floor(angleDegree)) + 179

            # Limitation of the angular position's command
            if angleByte < 60:
                angleByte = 60
            if angleByte > 180:
                angleByte = 180

            # Filling the list of the 3 angle values
            angles.append(angleByte)
        # The controller board of the real robot receives `angles` values
        self.serialport.packetOut = angles


def createScene(rootNode):
    from stlib3.scene import Scene
    scene = Scene(rootNode, gravity=[0., -9810., 0.], dt=0.01, iterative=False,
                  plugins=["SofaSparseSolver", "SofaOpenglVisual", "SofaSimpleFem", "SoftRobots",
                           'SofaBoundaryCondition', 'SofaDeformable', 'SofaEngine', 'SofaGeneralRigid',
                           'SofaMiscMapping', 'SofaRigid', 'SofaGraphComponent', 'SofaGeneralAnimationLoop', 'SofaGeneralEngine'])

    # Adding contact handling
    scene.addMainHeader()
    scene.addObject('AttachBodyButtonSetting', stiffness=10)  # Set mouse spring stiffness
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')

    scene.VisualStyle.displayFlags = "showBehavior"

    tripod = scene.Modelling.addChild(Tripod())

    serial = SerialPortBridgeGeneric(scene)

    # The real robot receives data from the 3 actuators
    serialportctrl = scene.addObject(
        SerialPortController(name="SerialPortController", inputs=tripod.actuatedarms, serialport=serial))

    # The simulation's control with keystrokes is still available
    controller = scene.addObject(
        TripodControllerWithCom(scene, actuators=tripod.actuatedarms, serialportctrl=serialportctrl))
    # You can set the animation from the python script by adding this call
    controller.initTripod('A')

    scene.Simulation.addChild(tripod.RigidifiedStructure)

    motors = scene.Simulation.addChild("Motors")
    for i in range(3):
        motors.addChild(tripod.getChild('ActuatedArm' + str(i)))

    # Temporary additions to have the system correctly built in SOFA
    # Will no longer be required in SOFA v22.06
    scene.Simulation.addObject('MechanicalMatrixMapper',
                               name="mmmFreeCenter",
                               template='Vec3,Rigid3',
                               object1="@RigidifiedStructure/DeformableParts/dofs",
                               object2="@RigidifiedStructure/FreeCenter/dofs",
                               nodeToParse="@RigidifiedStructure/DeformableParts/ElasticMaterialObject")

    for i in range(3):
        scene.Simulation.addObject('MechanicalMatrixMapper',
                                   name="mmmDeformableAndArm" + str(i),
                                   template='Vec1,Vec3',
                                   object1="@Modelling/Tripod/ActuatedArm" + str(i) + "/ServoMotor/Articulation/dofs",
                                   object2="@Simulation/RigidifiedStructure/DeformableParts/dofs",
                                   skipJ2tKJ2=True,
                                   nodeToParse="@Simulation/RigidifiedStructure/DeformableParts/ElasticMaterialObject")
