import Sofa
from splib3.numerics import Quat
from splib3.constants import Key
from splib3.interface import serialport

import numpy as np
import math
import serial
import time
import csv


class EffectorController(Sofa.Core.Controller):
    """The goal of this controller is to :
       - control the orientation of the goal
    """

    def __init__(self, *args, serialport=None, servomotors=None, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "EffectorController"
        self.referenceNode = args[1]

    def onKeypressedEvent(self, event):
        key = event['key']

        positionRigid = np.array(self.referenceNode.goalMO.position.value)
        position = positionRigid[0][0:3]
        quat = Quat(positionRigid[0][3:7])
        angles = quat.getEulerAngles(axes='sxyz')

        # rotate around x
        if key == Key.uparrow:
            self.index = 0

        # rotate around z
        if key == Key.rightarrow:
            self.index = 2

        # increase or decrease angle of 0.1 rad (5.7deg)
        if key == Key.plus:
            angles[self.index] += 0.1
        if key == Key.minus:
            angles[self.index] -= 0.1

        new_quat = Quat.createFromEuler(angles)
        self.referenceNode.goalMO.position.value = [
            list(position) + [new_quat.take(0), new_quat.take(1), new_quat.take(2), new_quat.take(3)]]


class CloseLoopController(Sofa.Core.Controller):
    """The goal of this controller it to :
        - add a gain
        - add an integrator
        - add an antiwindup
    """

    def __init__(self, *args, serialport=None, servomotors=None, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "CloseLoopController"
        self.nodeGoal = args[1]
        self.referenceNode = args[2]
        self.arduino = args[3]

        # parameters for the Proportionnal controller
        self.time = time.time()
        self.t = [0]

        # controller parameters:
        # TODO update parameters
        self.ki = 0
        self.kp = 0
        self.sat = 0
        self.kb = 0

        self.reference = [0, 0]
        self.command = [0, 0]
        self.command_sat = [0, 0]
        self.measure = [0, 0]

        self.integrator_term = [0, 0]

        self.file = open('data/results/closedLoop.csv', 'w')
        writer = csv.writer(self.file)
        writer.writerow(['time', 'x_reference', 'x_command', 'x_measure', 'z_reference', 'z_command', 'z_measure'])

    ########################################
    # Proportionnal controller functions
    ########################################

    def controller(self):
        """ compute self.command_sat (2 dimensions vector)"""
        # TODO PI Controller for each axis
        # compute error
        # compute proportionnal term
        # compute integrator term
        # compute command_sat
        # optionnal compute antiwindup

    ########################################
    # other functions
    ########################################

    def onAnimateBeginEvent(self, e):
        # get real time step
        t2 = time.time()
        self.dt = t2 - self.time
        self.time = t2
        self.t.append(self.t[-1] + self.dt)

        # get new reference
        q = Quat(self.referenceNode.goalMO.position.value[0][3:7])
        angles_target = q.getEulerAngles(axes='sxyz')
        self.reference = [angles_target[0], angles_target[2]]

        # get sensor value
        self.measure = self.arduino.sensor

        # get new ouput value
        self.controller()

        # write command in node goal
        positionRigid = np.array(self.nodeGoal.goalMO.position.value)
        position = positionRigid[0][0:3]
        quat = Quat(positionRigid[0][3:7])
        angles = quat.getEulerAngles(axes='sxyz')
        new_quat = Quat.createFromEuler([self.command_sat[0], angles[1], self.command_sat[1]])
        self.nodeGoal.goalMO.position.value = [
            list(position) + [new_quat.take(0), new_quat.take(1), new_quat.take(2), new_quat.take(3)]]

        row = [self.t[-1], self.reference[0], self.command_sat[0], self.measure[0], self.reference[1],
               self.command_sat[1], self.measure[1]]

        # create the csv writer
        writer = csv.writer(self.file)

        # write a row to the csv file
        writer.writerow(row)


class InverseController(Sofa.Core.Controller):
    """This controller has two role:
       - if user press I inverse kinematics is started
       - if state is in comm, send and receive data from arduino
       - state is change by DirectController
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        # CHANGE HERE the serialport that correspond to your computer
        # self.serialObj = serial.Serial("COM3", 57600, timeout=0.05)
        # self.serialObj = serial.Serial("/dev/cu.usbserial-1420", 57600, timeout=0.05)
        # self.serialObj = serial.Serial("/dev/ttyACM0", 57600, timeout=0.05)
        self.serialObj = serial.Serial(serialport.getDevicePort('Arduino', method='manufacturer'), 57600, timeout=0.05)
        self.nodeTripod = args[1]
        self.nodesInverseComponents = args[2]
        self.state = "init"
        self.sensor = [0, 0]
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

        # read serial port
        currentLine = self.serialObj.readline()
        try:
            DecodedAndSplit = currentLine.decode().split(',')
            self.sensor = [float(angle) * math.pi / 180 for angle in DecodedAndSplit[:2]]
        except:
            print("Error while decoding/writing IMU data")

        # write convert angles in byte
        if self.activate:

            Angles = [0] * 3
            for i in range(3):
                Angles[i] = self.nodeTripod.actuatedarms[i].ServoMotor.Articulation.dofs.position[0][0]

            AnglesOut = []

            for i in range(3):
                # Conversion of the angle values from radians to degrees
                angleDegree = Angles[i] * 360 / (2.0 * math.pi)
                angleByte = int(math.floor(angleDegree)) + 179

                # Limitation of the angular position's command
                if angleByte < 60:
                    angleByte = 60
                if angleByte > 180:
                    angleByte = 180

                # Filling the list with the 3 angle values
                AnglesOut.append(angleByte)

        # write to serial port
        String = str(AnglesOut[0]) + ' ' + str(AnglesOut[1]) + ' ' + str(AnglesOut[2]) + '\n'
        ByteString = String.encode('ASCII')
        self.serialObj.write(ByteString)
