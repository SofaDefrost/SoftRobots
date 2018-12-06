#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
import math

class controller(Sofa.PythonScriptController):





    def initGraph(self, node):

            self.node = node
            self.finger1Node=self.node.getChild('finger')
            self.pressureConstraint1Node = self.finger1Node.getChild('cavity')

    def onKeyPressed(self,c):
            self.dt = self.node.findData('dt').value
            incr = self.dt*1000.0;

            self.MecaObject1=self.finger1Node.getObject('tetras');

            self.pressureConstraint1 = self.pressureConstraint1Node.getObject('SurfacePressureConstraint')

            if (c == "+"):
                pressureValue = self.pressureConstraint1.findData('value').value[0][0] + 0.01
                if pressureValue > 1.5:
                    pressureValue = 1.5
                self.pressureConstraint1.findData('value').value = str(pressureValue)

            if (c == "-"):
                pressureValue = self.pressureConstraint1.findData('value').value[0][0] - 0.01
                if pressureValue < 0:
                    pressureValue = 0
                self.pressureConstraint1.findData('value').value = str(pressureValue)
