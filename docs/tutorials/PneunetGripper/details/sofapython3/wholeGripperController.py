#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
from Sofa.constants import *
import math


def moveRestPos(rest_pos, dx, dy, dz):
    out = []
    for i in range(0,len(rest_pos)) :
        out += [[rest_pos[i][0]+dx, rest_pos[i][1]+dy, rest_pos[i][2]+dz]]
    return out

def rotateRestPos(rest_pos,rx,centerPosY,centerPosZ):
    out = []
    for i in range(0,len(rest_pos)) :
        newRestPosY = (rest_pos[i][1] - centerPosY)*math.cos(rx) - (rest_pos[i][2] - centerPosZ)*math.sin(rx) +  centerPosY
        newRestPosZ = (rest_pos[i][1] - centerPosY)*math.sin(rx) + (rest_pos[i][2] - centerPosZ)*math.cos(rx) +  centerPosZ
        out += [[rest_pos[i][0], newRestPosY, newRestPosZ]]
    return out

class WholeGripperController(Sofa.Core.Controller):

    def __init__(self, *a, **kw):

        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.finger1Node=self.node.getChild('finger1')
        self.finger2Node=self.node.getChild('finger2')
        self.finger3Node=self.node.getChild('finger3')
        self.pressureConstraint1Node = self.finger1Node.getChild('cavity')
        self.pressureConstraint2Node = self.finger2Node.getChild('cavity')
        self.pressureConstraint3Node = self.finger3Node.getChild('cavity')

        self.centerPosY = 70
        self.centerPosZ = 0
        self.rotAngle = 0

        return

    def onKeypressedEvent(self,e):
            self.dt = self.node.findData('dt').value
            incr = self.dt*1000.0;

            self.MecaObject1=self.finger1Node.tetras;
            self.MecaObject2=self.finger2Node.tetras;
            self.MecaObject3=self.finger3Node.tetras;

            self.pressureConstraint1 = self.pressureConstraint1Node.SurfacePressureConstraint
            self.pressureConstraint2 = self.pressureConstraint2Node.SurfacePressureConstraint
            self.pressureConstraint3 = self.pressureConstraint3Node.SurfacePressureConstraint

            if (e["key"] == Sofa.constants.Key.plus):
                print('squeezing...')
                pressureValue = self.pressureConstraint1.findData('value').value[0] + 0.01
                if pressureValue > 1.5:
                    pressureValue = 1.5
                self.pressureConstraint1.findData('value').value = [pressureValue]
                pressureValue = self.pressureConstraint2.findData('value').value[0] + 0.01
                if pressureValue > 1.5:
                    pressureValue = 1.5
                self.pressureConstraint2.findData('value').value = [pressureValue]
                pressureValue = self.pressureConstraint3.findData('value').value[0] + 0.01
                if pressureValue > 1.5:
                    pressureValue = 1.5
                self.pressureConstraint3.findData('value').value = [pressureValue]

            elif (e["key"] == Sofa.constants.Key.minus):
                print('releasing...')
                pressureValue = self.pressureConstraint1.findData('value').value[0] - 0.01
                self.pressureConstraint1.findData('value').value = [pressureValue]
                pressureValue = self.pressureConstraint2.findData('value').value[0] - 0.01
                self.pressureConstraint2.findData('value').value = [pressureValue]
                pressureValue = self.pressureConstraint3.findData('value').value[0] - 0.01
                self.pressureConstraint3.findData('value').value = [pressureValue]

            # UP key :
            elif (e["key"] == Sofa.constants.Key.uparrow):
                test1 = moveRestPos(self.MecaObject1.rest_position.value, 3.0, 0.0, 0.0)
                self.MecaObject1.rest_position.value = test1
                test2 = moveRestPos(self.MecaObject2.rest_position.value, 3.0, 0.0, 0.0)
                self.MecaObject2.rest_position.value = test2
                test3 = moveRestPos(self.MecaObject3.rest_position.value, 3.0, 0.0, 0.0)
                self.MecaObject3.rest_position.value = test3



            # DOWN key : rear
            elif (e["key"] == Sofa.constants.Key.downarrow):
                test = moveRestPos(self.MecaObject1.rest_position.value, -3.0, 0.0, 0.0)
                self.MecaObject1.rest_position.value = test
                test = moveRestPos(self.MecaObject2.rest_position.value, -3.0, 0.0, 0.0)
                self.MecaObject2.rest_position.value = test
                test = moveRestPos(self.MecaObject3.rest_position.value, -3.0, 0.0, 0.0)
                self.MecaObject3.rest_position.value = test


            # LEFT key : left
            elif (e["key"] == Sofa.constants.Key.leftarrow):
                dy = 3.0*math.cos(self.rotAngle)
                dz = 3.0*math.sin(self.rotAngle)
                test = moveRestPos(self.MecaObject1.rest_position.value, 0.0, dy, dz)
                self.MecaObject1.rest_position.value = test
                test = moveRestPos(self.MecaObject2.rest_position.value, 0.0, dy, dz)
                self.MecaObject2.rest_position.value = test
                test = moveRestPos(self.MecaObject3.rest_position.value, 0.0, dy, dz)
                self.MecaObject3.rest_position.value = test
                self.centerPosY = self.centerPosY + dy
                self.centerPosZ = self.centerPosZ + dz

            # RIGHT key : right
            elif (e["key"] == Sofa.constants.Key.rightarrow):
                dy = -3.0*math.cos(self.rotAngle)
                dz = -3.0*math.sin(self.rotAngle)
                test = moveRestPos(self.MecaObject1.rest_position.value, 0.0, dy, dz)
                self.MecaObject1.rest_position.value = test
                test = moveRestPos(self.MecaObject2.rest_position.value, 0.0, dy, dz)
                self.MecaObject2.rest_position.value = test
                test = moveRestPos(self.MecaObject3.rest_position.value, 0.0, dy, dz)
                self.MecaObject3.rest_position.value = test
                self.centerPosY = self.centerPosY + dy
                self.centerPosZ = self.centerPosZ + dz


            # a key : direct rotation
            elif (e["key"] == "A"):
                test = rotateRestPos(self.MecaObject1.rest_position.value, math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject1.rest_position.value = test
                test = rotateRestPos(self.MecaObject2.rest_position.value, math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject2.rest_position.value = test
                test = rotateRestPos(self.MecaObject3.rest_position.value, math.pi/16,self.centerPosY,self.centerPosZ)
                self.MecaObject3.rest_position.value = test
                self.rotAngle = self.rotAngle + math.pi/16

            # q key : indirect rotation
            elif (e["key"] == "Q"):
                test = rotateRestPos(self.MecaObject1.rest_position.value, -math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject1.rest_position.value = test
                test = rotateRestPos(self.MecaObject2.rest_position.value, -math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject2.rest_position.value = test
                test = rotateRestPos(self.MecaObject3.rest_position.value, -math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject3.rest_position.value = test
                self.rotAngle = self.rotAngle - math.pi/16
