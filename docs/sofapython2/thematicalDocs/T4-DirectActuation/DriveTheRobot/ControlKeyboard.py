#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa

class controller(Sofa.PythonScriptController):

    def initGraph(self, node):

        self.node = node
        self.increment = 5
        self.restPositionNodes = []
        self.previousValues = [[0,0,0],[0,0,0],[0,0,0]]

        for i in range(3):
            self.restPositionNodes.append(self.node.getChild('RestPositionLeg'+str(i)))

    def onKeyPressed(self,c):

        currentValues = []
        for i in range(len(self.restPositionNodes)):
            currentValues.append(self.restPositionNodes[i].getObject('meca'+str(i)).translation[0])

        resetPreviousValues = 0
        ###Move
        if (ord(c) == 18):  #  <--

            currentValues[0][2] = self.increment
            currentValues[1][2] = self.increment
            currentValues[2][2] = -self.increment

        elif (ord(c) == 19): # -->

            currentValues[0][2] = -self.increment
            currentValues[1][2] = self.increment
            currentValues[2][2] = self.increment

        elif (ord(c) == 20): #  up

            currentValues[0][2] = self.increment
            currentValues[1][2] = -self.increment
            currentValues[2][2] = self.increment

        elif (ord(c) == 21): # down == reset init pose

            currentValues[0][2] = -self.previousValues[0][2]
            currentValues[1][2] = -self.previousValues[1][2]
            currentValues[2][2] = -self.previousValues[2][2]

            resetPreviousValues = 1


        elif (c == "+"):

            currentValues[0][2] = self.increment
            currentValues[1][2] = self.increment
            currentValues[2][2] = self.increment

        elif (c == "-"):

            currentValues[0][2] =- self.increment
            currentValues[1][2] =- self.increment
            currentValues[2][2] =- self.increment

        if not resetPreviousValues:
            for i in range(0,3):
                self.previousValues[i][2]+=currentValues[i][2]
        else:
            for i in range(0,3):
                self.previousValues[i][2]=0

        for i in range(len(self.restPositionNodes)):

            self.restPositionNodes[i].getObject('meca'+str(i)).findData("translation").value =  currentValues[i]
            self.restPositionNodes[i].getObject('meca'+str(i)).reinit()
