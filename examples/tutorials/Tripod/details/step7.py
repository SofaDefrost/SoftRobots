# -*- coding: utf-8 -*-
"""
Step 7: Here we are showing how to setup a communication with the servomotors
"""
import Sofa
from tutorial import *
from splib3.animation import animate
from splib3.constants import Key
from splib3.interface import serialport
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
            angleDegree = arm.ServoMotor.angleOut.value * 360 / (2.0 * pi)
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

    pluginList = ["ArticulatedSystemPlugin",
                  "Sofa.Component.AnimationLoop",
                  "Sofa.Component.Constraint.Lagrangian.Correction",
                  "Sofa.Component.Constraint.Lagrangian.Solver",
                  "Sofa.Component.Constraint.Projective",
                  "Sofa.Component.Engine.Select",
                  "Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.Mapping.MappedMatrix",
                  "Sofa.Component.Mass",
                  "Sofa.Component.SolidMechanics.FEM.Elastic",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Constant",
                  "Sofa.Component.Topology.Container.Dynamic",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.GUI.Component",
                  "SoftRobots", "Sofa.Component.Mapping.Linear", "Sofa.Component.Mapping.NonLinear"]

    scene = Scene(rootNode, gravity=[0., -9810., 0.], dt=0.01, iterative=False,
                  plugins=pluginList)

    # Adding contact handling
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.VisualStyle.displayFlags = "showBehavior"
    scene.Settings.mouseButton.stiffness = 10

    tripod = scene.Modelling.addChild(Tripod())

    serial = SerialPortBridgeGeneric(scene, serialport=serialport.getDevicePort('Arduino', method='manufacturer'))

    # The real robot receives data from the 3 actuators
    serialportctrl = scene.addObject(
        SerialPortController(name="SerialPortController", inputs=tripod.actuatedarms, serialport=serial))

    # The simulation's control with keystrokes is still available
    controller = scene.addObject(
        TripodControllerWithCom(scene, actuators=tripod.actuatedarms, serialportctrl=serialportctrl))
    # You can set the animation from the python script by adding this call
    controller.initTripod('A')

    scene.Simulation.addChild(tripod)
