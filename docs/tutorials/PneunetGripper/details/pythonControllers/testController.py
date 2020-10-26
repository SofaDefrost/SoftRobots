#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
import math


def moveRestPos(rest_pos, dx, dy, dz):
    str_out = ' '
    for i in xrange(0,len(rest_pos)) :
        str_out= str_out + ' ' + str(rest_pos[i][0]+dx)
        str_out= str_out + ' ' + str(rest_pos[i][1]+dy)
        str_out= str_out + ' ' + str(rest_pos[i][2]+dz)
    return str_out

def rotateRestPos(rest_pos,rx,centerPosY,centerPosZ):
    str_out = ' '
    for i in xrange(0,len(rest_pos)) :
        newRestPosY = (rest_pos[i][1] - centerPosY)*math.cos(rx) - (rest_pos[i][2] - centerPosZ)*math.sin(rx) +  centerPosY
        newRestPosZ = (rest_pos[i][1] - centerPosY)*math.sin(rx) + (rest_pos[i][2] - centerPosZ)*math.cos(rx) +  centerPosZ
        str_out= str_out + ' ' + str(rest_pos[i][0])
        str_out= str_out + ' ' + str(newRestPosY)
        str_out= str_out + ' ' + str(newRestPosZ)
    return str_out

class GripperController(Sofa.PythonScriptController):

    def __init__(self, node):

            self.listening = True
            self.name = "GripperController"
            self.node = node

    def initGraph(self, node):

            self.finger1Node=self.node.getChild('finger1')
            self.finger2Node=self.node.getChild('finger2')
            self.finger3Node=self.node.getChild('finger3')
            self.pressureConstraint1Node = self.finger1Node.getChild('cavity')
            self.pressureConstraint2Node = self.finger2Node.getChild('cavity')
            self.pressureConstraint3Node = self.finger3Node.getChild('cavity')

            self.centerPosY = 70
            self.centerPosZ = 0
            self.rotAngle = 0

    def onKeyPressed(self,c):
            self.dt = self.node.findData('dt').value
            incr = self.dt*1000.0;

            self.MecaObject1=self.finger1Node.getObject('dofs');
            self.MecaObject2=self.finger2Node.getObject('dofs');
            self.MecaObject3=self.finger3Node.getObject('dofs');

            self.pressureConstraint1 = self.pressureConstraint1Node.getObject('SurfacePressureConstraint')
            self.pressureConstraint2 = self.pressureConstraint2Node.getObject('SurfacePressureConstraint')
            self.pressureConstraint3 = self.pressureConstraint3Node.getObject('SurfacePressureConstraint')

            if (c == "+"):
                print 'squeezing...'
                pressureValue = self.pressureConstraint1.findData('value').value[0][0] + 0.01
                if pressureValue > 1.5:
                    pressureValue = 1.5
                self.pressureConstraint1.findData('value').value = str(pressureValue)
                pressureValue = self.pressureConstraint2.findData('value').value[0][0] + 0.01
                if pressureValue > 1.5:
                    pressureValue = 1.5
                self.pressureConstraint2.findData('value').value = str(pressureValue)
                pressureValue = self.pressureConstraint3.findData('value').value[0][0] + 0.01
                if pressureValue > 1.5:
                    pressureValue = 1.5
                self.pressureConstraint3.findData('value').value = str(pressureValue)

            if (c == "-"):
                print 'releasing...'
                pressureValue = self.pressureConstraint1.findData('value').value[0][0] - 0.01
                self.pressureConstraint1.findData('value').value = str(pressureValue)
                pressureValue = self.pressureConstraint2.findData('value').value[0][0] - 0.01
                self.pressureConstraint2.findData('value').value = str(pressureValue)
                pressureValue = self.pressureConstraint3.findData('value').value[0][0] - 0.01
                self.pressureConstraint3.findData('value').value = str(pressureValue)

            # UP key :
            if ord(c)==20:
                test1 = moveRestPos(self.MecaObject1.rest_position, 3.0, 0.0, 0.0)
                self.MecaObject1.findData('rest_position').value = test1
                test2 = moveRestPos(self.MecaObject2.rest_position, 3.0, 0.0, 0.0)
                self.MecaObject2.findData('rest_position').value = test2
                test3 = moveRestPos(self.MecaObject3.rest_position, 3.0, 0.0, 0.0)
                self.MecaObject3.findData('rest_position').value = test3



            # DOWN key : rear
            if ord(c)==18:
                test = moveRestPos(self.MecaObject1.rest_position, -3.0, 0.0, 0.0)
                self.MecaObject1.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject2.rest_position, -3.0, 0.0, 0.0)
                self.MecaObject2.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject3.rest_position, -3.0, 0.0, 0.0)
                self.MecaObject3.findData('rest_position').value = test


            # LEFT key : left
            if ord(c)==19:
                dy = 3.0*math.cos(self.rotAngle)
                dz = 3.0*math.sin(self.rotAngle)
                test = moveRestPos(self.MecaObject1.rest_position, 0.0, dy, dz)
                self.MecaObject1.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject2.rest_position, 0.0, dy, dz)
                self.MecaObject2.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject3.rest_position, 0.0, dy, dz)
                self.MecaObject3.findData('rest_position').value = test
                self.centerPosY = self.centerPosY + dy
                self.centerPosZ = self.centerPosZ + dz

            # RIGHT key : right
            if ord(c)==21:
                dy = -3.0*math.cos(self.rotAngle)
                dz = -3.0*math.sin(self.rotAngle)
                test = moveRestPos(self.MecaObject1.rest_position, 0.0, dy, dz)
                self.MecaObject1.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject2.rest_position, 0.0, dy, dz)
                self.MecaObject2.findData('rest_position').value = test
                test = moveRestPos(self.MecaObject3.rest_position, 0.0, dy, dz)
                self.MecaObject3.findData('rest_position').value = test
                self.centerPosY = self.centerPosY + dy
                self.centerPosZ = self.centerPosZ + dz

            # a key : direct rotation
            if (ord(c) == 65):
                test = rotateRestPos(self.MecaObject1.rest_position, math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject1.findData('rest_position').value = test
                test = rotateRestPos(self.MecaObject2.rest_position, math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject2.findData('rest_position').value = test
                test = rotateRestPos(self.MecaObject3.rest_position, math.pi/16,self.centerPosY,self.centerPosZ)
                self.MecaObject3.findData('rest_position').value = test
                self.rotAngle = self.rotAngle + math.pi/16

            # q key : indirect rotation
            if (ord(c) == 81):
                test = rotateRestPos(self.MecaObject1.rest_position, -math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject1.findData('rest_position').value = test
                test = rotateRestPos(self.MecaObject2.rest_position, -math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject2.findData('rest_position').value = test
                test = rotateRestPos(self.MecaObject3.rest_position, -math.pi/16, self.centerPosY,self.centerPosZ)
                self.MecaObject3.findData('rest_position').value = test
                self.rotAngle = self.rotAngle - math.pi/16
