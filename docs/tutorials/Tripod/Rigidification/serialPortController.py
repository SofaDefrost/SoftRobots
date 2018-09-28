#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa

class interface(Sofa.PythonScriptController):


    def initGraph(self, node):
        self.node = node
        self.node.getObject('serial').findData('sentData').value = [150,150,150]


    def onEndAnimationStep(self,dt):


        displacementA0 = self.node.getChild('robot').getChild('Rigid').getObject('SlidingActuator1').findData("displacement").value*180/3.14 + 93
        displacementA1 = self.node.getChild('robot').getChild('Rigid').getObject('SlidingActuator2').findData("displacement").value*180/3.14 + 93
        displacementA2 = self.node.getChild('robot').getChild('Rigid').getObject('SlidingActuator3').findData("displacement").value*180/3.14 + 93


        outputVector = [displacementA0, displacementA1, displacementA2]


        self.node.getObject('serial').findData('sentData').value = outputVector
        #print(outputVector)
