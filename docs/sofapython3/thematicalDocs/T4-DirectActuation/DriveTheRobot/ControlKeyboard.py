#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key


class ControlKeyboard(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.increment = 5
        self.previousValues = [[0,0,0],[0,0,0],[0,0,0]]

    def onKeypressedEvent(self,e):
        currentValues = []
        for i in range(3):
            translation = self.node.getChild('RestPositionLeg'+str(i)).getObject('meca'+str(i)).translation.value
            currentValues.append(list(translation))

        resetPreviousValues = 0
        # Move
        if e['key']==Key.leftarrow:  # <--

            currentValues[0][2] = self.increment
            currentValues[1][2] = self.increment
            currentValues[2][2] = -self.increment

        elif e['key']==Key.rightarrow:  # -->

            currentValues[0][2] = -self.increment
            currentValues[1][2] = self.increment
            currentValues[2][2] = self.increment

        elif e['key']==Key.uparrow:  # up

            currentValues[0][2] = self.increment
            currentValues[1][2] = -self.increment
            currentValues[2][2] = self.increment

        elif e['key']==Key.downarrow:  # down == reset init pose

            currentValues[0][2] = -self.previousValues[0][2]
            currentValues[1][2] = -self.previousValues[1][2]
            currentValues[2][2] = -self.previousValues[2][2]

            resetPreviousValues = 1

        elif e['key']==Key.plus:

            currentValues[0][2] = self.increment
            currentValues[1][2] = self.increment
            currentValues[2][2] = self.increment

        elif e['key']==Key.minus:

            currentValues[0][2] = -self.increment
            currentValues[1][2] = -self.increment
            currentValues[2][2] = -self.increment

        if not resetPreviousValues:
            for i in range(0,3):
                self.previousValues[i][2]+=currentValues[i][2]
        else:
            for i in range(0,3):
                self.previousValues[i][2]=0

        for i in range(3):
            self.node.getChild('RestPositionLeg'+str(i)).getObject('meca'+str(i)).findData("translation").value = currentValues[i]
            self.node.getChild('RestPositionLeg'+str(i)).getObject('meca'+str(i)).reinit()
