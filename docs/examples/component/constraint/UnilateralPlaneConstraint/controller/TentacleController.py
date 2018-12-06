#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
import matplotlib.pyplot as plt

class controller(Sofa.PythonScriptController):
         
    def initGraph(self, node):

            self.node = node
            
    def reset(self):

        displacement = 0
	self.node.getChild('tentacle').getChild('actuator').getObject('cable').findData('value').value = str(displacement)


    def onBeginAnimationStep(self, dt):

        displacement = self.node.getChild('tentacle').getChild('actuator').getObject('cable').findData('value').value[0][0]

        if(displacement < 35):
            displacement += 0.05
            self.node.getChild('tentacle').getChild('actuator').getObject('cable').findData('value').value = str(displacement)
            print "displacement = "+str(displacement)

