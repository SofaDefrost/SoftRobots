from tripodcontroller import TripodController, setupanimation
from splib.animation import animate
from splib.constants import Key
from tripod import Tripod
import Sofa
import math


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
        TripodController.__init__(self, node, actuators)
        self.serialportctrl = serialportctrl

    def initTripod(self, key):
        if key == Key.A and self.serialportctrl.state == "init":
            self.serialportctrl.state = "no-comm"
            animate(setupanimation, {"actuators": self.actuators, "step": 35.0, "angularstep": -1.4965}, duration=0.2)

        # Inclusion of the keystroke to start data sending = establishing communication ('comm')
        if key == Key.B and self.serialportctrl.state == "no-comm":
            self.serialportctrl.state = "comm"


# Description of how the communication is handled
def SerialPortBridgeGeneric(rootNode, serialport="/dev/ttyUSB0"):
    return rootNode.createObject("SerialPortBridgeGeneric", port=serialport, baudRate=115200, size=3, listening=True, header=255)


# Data sending controller
class SerialPortController(Sofa.PythonScriptController):
    def __init__(self, node, inputs, serialport):
        self.name = "serialportcontroller"
        self.actuatedarms = inputs
        self.serialport = serialport
        self.serialport.packetOut = [150, 150, 150]
        self.state = "init"

    def onEndAnimationStep(self, dt):
        # Data sending if the robot is initializing or in the no-communication sate
        if self.state == "init":
            return

        if self.state == "no-comm":
            return

        # Vector storing the simulated servomotors' angular position
        angles = []

        for arm in self.actuatedarms:
            # Conversion of the angle values from radians to degrees
            angleDegree = arm.ServoMotor.angleOut*360/(2.0*math.pi)
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
