# -*- coding: utf-8 -*-
from stlib3.scene import Scene as stScene, ContactHeader
from splib3.objectmodel import setData
from splib3.loaders import getLoadingLocation
import Sofa
from math import *


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


def visit(target, fct):
    if isinstance(target, Sofa.Node):
        for child in target.getChildren():
            visit(child, fct)
        for obj in target.getObjects():
            fct(obj)
    fct(target)


def loadPosition(root, filename):
    import json

    with open(filename, "rt") as file:
        data = json.load(file)

        def setMatchingData(target):
            for field in target.getListOfDataFields():
                if field.getLinkPath() in data:
                    field.value = data[field.getLinkPath()]

        for k, v in enumerate(data):
            visit(root, setMatchingData)


def Modelling(parent):
    """add an empty node for modelling"""
    modeling = parent.addChild("Modelling")
    return modeling


def Simulation(parent):
    """add an empty node for simulation"""
    simulation = parent.addChild("Simulation")
    simulation.addObject("EulerImplicitSolver")
    simulation.addObject("CGLinearSolver", iterations=250, tolerance=1e-20, threshold=1e-20)
    return simulation


def Scene(parent, **kwargs):
    import os
    if "plugins" not in kwargs:
        kwargs["plugins"] = []

    kwargs["plugins"].append("SofaSparseSolver")

    scene = stScene(parent, **kwargs)
    setData(scene, dt=0.025)
    setData(scene, gravity=[0., -9810., 0.])
    setData(scene.Settings.VisualStyle, displayFlags="showBehavior showForceFields")

    Modelling(scene)
    Simulation(scene)
    parent.addObject("FreeMotionAnimationLoop")
    parent.addObject("GenericConstraintSolver", maxIterations=250, tolerance=1e-20)

    ctx = scene.Config
    ctx.addObject("MeshSTLLoader", name="loader", filename=getLoadingLocation("data/mesh/blueprint.stl", __file__))
    ctx.addObject("OglModel", src="@loader")
    ctx.addObject("AddDataRepository", path=os.path.abspath(os.path.dirname(__file__)))

    return parent


# Description of how the communication is handled
def SerialPortBridgeGeneric(rootNode, serialport="/dev/ttyUSB0"):
    return rootNode.addObject("SerialPortBridgeGeneric", port=serialport, baudRate=115200, size=3, listening=True,
                              header=255)


# Data sending controller
class SerialPortController(Sofa.Core.Controller):
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
            angleDegree = arm.ServoMotor.angleOut * 360 / (2.0 * pi)
            angleByte = int(floor(angleDegree)) + 179

            # Limitation of the angular position's command
            if angleByte < 60:
                angleByte = 60
            if angleByte > 180:
                angleByte = 180

            # Filling the list with the 3 angle values
            angles.append(angleByte)

        # The controller board of the real robot receives `angles` values
        self.serialport.packetOut = angles


def addContact(self, alarmDistance=5, contactDistance=2, frictionCoef=0.0):
    ContactHeader(self, alarmDistance, contactDistance, frictionCoef)
