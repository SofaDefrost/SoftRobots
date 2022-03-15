#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
import math


class controller(Sofa.PythonScriptController):


    def initGraph(self, node):
        self.node  = node
        self.leg1  = node.getChild('robot').getChild('leg1').getObject("sa")
        self.leg2  = node.getChild('robot').getChild('leg2').getObject("sa")
        self.leg3  = node.getChild('robot').getChild('leg3').getObject("sa")

        outputVector = [0., 0., 0.]
        self.node.getObject('serial').findData('sentData').value = outputVector


    def resetGraph(self, node):
        outputVector = [0., 0., 0.]
        self.node.getObject('serial').findData('sentData').value = outputVector


    def onEndAnimationStep(self,dt):

        displacementLeg1  = self.leg1.findData("displacement").value
        displacementLeg2  = self.leg2.findData("displacement").value
        displacementLeg3  = self.leg3.findData("displacement").value
        outputVector = [displacementLeg1, displacementLeg2, displacementLeg3]

        self.node.getObject('serial').findData('sentData').value = outputVector
