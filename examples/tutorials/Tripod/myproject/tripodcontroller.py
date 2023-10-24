import Sofa
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key
from stlib3.scene import Scene
from tripod import Tripod
import math


def dumpPosition(fields, filename):
    import json
    data = {}
    for field in fields:
        v = field.value
        if isinstance(v, list) and isinstance(v[0], list):
            v = v[0]
        data[field.getLinkPath()] = v
    with open(filename, "w+t") as file:
        json.dump(data, file)


def setupanimation(actuators, step, angularstep, factor):
    """This function is called repeatidely in an animation.
       It moves the actuators by translating & rotating them according to the factor
       value.
    """
    for actuator in actuators:
        rigid = RigidDof(actuator.ServoMotor.ServoBody.dofs)
        rigid.setPosition(rigid.rest_position + rigid.forward * step * factor)
        actuator.angleIn = angularstep * factor


def saveTripodPosition(actuators, step, angularstep, factor):
    t = []
    for actuator in actuators:
        t.append(actuator.ServoMotor.ServoBody.dofs.position)
        t.append(actuator.ServoMotor.angleIn)

    dumpPosition(t, "tripodRestPosition.json")


class TripodController(Sofa.Core.Controller):
    """This controller has two roles:
       - if the user presses up/left/right/down/plus/minus, the servomotor angle
         is changed.
       - if thr user presses A, an animation is started to move the servomotor to the initial position
         of the real robot.
    """

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "TripodController"

        self.stepsize = 0.1
        self.actuators = kwargs["actuators"]

    def onKeypressedEvent(self, event):
        key = event['key']
        self.initTripod(key)
        self.animateTripod(key)

    def initTripod(self, key):
        if key == Key.A:
            animate(setupanimation,
                    {"actuators": self.actuators, "step": 35.0, "angularstep": -1.4965},
                    duration=0.2)

    def animateTripod(self, key):
        if key == Key.uparrow:
            self.actuators[0].ServoMotor.angleIn = self.actuators[0].ServoMotor.angleOut.value + self.stepsize
        elif key == Key.downarrow:
            self.actuators[0].ServoMotor.angleIn = self.actuators[0].ServoMotor.angleOut.value - self.stepsize

        if key == Key.leftarrow:
            self.actuators[1].ServoMotor.angleIn = self.actuators[1].ServoMotor.angleOut.value + self.stepsize
        elif key == Key.rightarrow:
            self.actuators[1].ServoMotor.angleIn = self.actuators[1].ServoMotor.angleOut.value - self.stepsize

        if key == Key.plus:
            self.actuators[2].ServoMotor.angleIn = self.actuators[2].ServoMotor.angleOut.value + self.stepsize
        elif key == Key.minus:
            self.actuators[2].ServoMotor.angleIn = self.actuators[2].ServoMotor.angleOut.value - self.stepsize


class TripodControllerWithCom(TripodController):
    """This controller has three roles:
       - if the user presses up/left/right/down/plus/minus, the servomotor angle
         is changed.
       - if thr user presses A, an animation is started to move the servomotor to the initial position
         of the real robot.
       - if thr user presses B start the communication with controller card, send
         servomotor commands
    """

    def __init__(self, node, actuators, serialportctrl):
        TripodController.__init__(self, node, actuators=actuators)
        self.name = "TripodControllerWithCom"
        self.serialportctrl = serialportctrl

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
# def SerialPortBridgeGeneric(rootNode, serialport='/dev/ttyACM0'):
def SerialPortBridgeGeneric(rootNode, serialport="/dev/ttyUSB0"):
    return rootNode.addObject("SerialPortBridgeGeneric", port=serialport, baudRate=115200, size=3, listening=True,
                              header=255)


# Data sending controller
class SerialPortController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "SerialPortController"
        self.actuatedarms = kwargs['inputs']
        self.serialport = kwargs['serialport']
        self.serialport.packetOut = [150, 150, 150]
        self.state = "init"

    def onEndAnimationEvent(self):
        # Data sending if the robot is initializing or in the no-communication sate
        if self.state == "init":
            return

        if self.state == "no-comm":
            return

        # Vector storing the simulated servomotors' angular position
        angles = []

        for arm in self.actuatedarms:
            # Conversion of the angle values from radians to degrees
            angleDegree = arm.ServoMotor.angleOut * 360 / (2.0 * math.pi)
            angleByte = int(math.floor(angleDegree)) + 179

            # Limitation of the angular position's command
            if angleByte < 60:
                angleByte = 60
            if angleByte > 180:
                angleByte = 180

            # Filling the list of the 3 angle values
            angles.append(angleByte)

        # The controller board of the real robot receives `angles` values
        self.serialport.packetOut = angles


class InverseController(Sofa.Core.Controller):
    """This controller has two role:
       - if user press up/left/right/down/plus/minus the servo motor angle
         is changed.
       - if user press A an animation is started to move the motor to the physical position
         they are occupying in the real robot.
    """

    def __init__(self, *args, serialport=None, servomotors=None, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "InverseController"
        self.nodeGoal = args[1]
        self.nodesInverseComponents = args[2]
        self.nodeDofRigid = args[3]
        self.nodeTripod = args[4]
        self.serialport = args[5]
        self.serialport.packetOut = [150, 150, 150]
        self.state = "init"
        self.actuators = servomotors
        self.activate = False

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.I:
            for i in range(3):
                self.nodeTripod.actuatedarms[i].ServoMotor.Articulation.RestShapeSpringsForceField.stiffness.value = [
                    0.]
            self.activate = True
            for node in self.nodesInverseComponents:
                node.activated = bool(self.activate)
                node.init()

    def onAnimateBeginEvent(self, e):

        if self.state == "init":
            return

        if self.state == "no-comm":
            return

        if (self.activate):
            # W_R_Dof = [0]*4;
            # W_R_Ref = [0]*4;
            # Ndir = [[ ]]*3;
            Angles = [0] * 3;

            Angles[0] = self.nodeTripod.actuatedarms[0].ServoMotor.Articulation.dofs.position[0][0];
            Angles[1] = self.nodeTripod.actuatedarms[1].ServoMotor.Articulation.dofs.position[0][0];
            Angles[2] = self.nodeTripod.actuatedarms[2].ServoMotor.Articulation.dofs.position[0][0];
            AnglesOut = [];
            for i in range(3):
                # Conversion of the angle values from radians to degrees
                angleDegree = Angles[i] * 360 / (2.0 * math.pi)
                angleByte = int(math.floor(angleDegree)) + 179

                # Limitation of the angular position's command
                if angleByte < 60:
                    angleByte = 60
                if angleByte > 180:
                    angleByte = 180

                # Filling the list of the 3 angle values
                AnglesOut.append(angleByte)

            # The controller board of the real robot receives `AnglesOut` values
            if (self.serialport):
                self.serialport.packetOut = [AnglesOut[0], AnglesOut[1], AnglesOut[2]]
                # self.serialport.packetOut = [int(AnglesDeg[0]+0), int(AnglesDeg[1]-0), int(AnglesDeg[2])]


class DirectController(Sofa.Core.Controller):
    """This controller has two role:
       - if user press up/left/right/down/plus/minus the servo motor angle
         is changed.
       - if user press A an animation is started to move the motor to the physical position
         they are occupying in the real robot.
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "DirectController"
        self.stepsize = 0.1
        self.actuators = args[1]
        self.serialportctrl = args[2]

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.A:  # and self.serialportctrl.state == "init":
            self.serialportctrl.state = "no-comm"
            animate(setupanimation, {"actuators": self.actuators, "step": 35.0, "angularstep": -1.4965}, duration=0.2)

        # Inclusion of the keystroke to start data sending = establishing communication ('comm')
        if key == Key.B and self.serialportctrl.state == "no-comm":
            self.serialportctrl.state = "comm"


def createScene(rootNode):
    scene = Scene(rootNode, iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    scene.VisualStyle.displayFlags = "showBehavior"

    tripod = scene.Modelling.addChild(Tripod())

    scene.addObject(TripodController(scene, actuators=[tripod.ActuatedArm0,
                                                       tripod.ActuatedArm1,
                                                       tripod.ActuatedArm2]))

    scene.Simulation.addChild(tripod.RigidifiedStructure)
    motors = scene.Simulation.addChild("Motors")
    motors.addChild(tripod.ActuatedArm0)
    motors.addChild(tripod.ActuatedArm1)
    motors.addChild(tripod.ActuatedArm2)
