#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa

class controller(Sofa.PythonScriptController):

    def initGraph(self, node):
        self.node = node

    def reset(self):
        displacement = 0
        self.node.tentacle.actuator.cable.findData('value').value = displacement

    def onBeginAnimationStep(self, dt):
        displacement = self.node.tentacle.actuator.cable.findData('value').value[0][0]

        if(displacement < 35):
            displacement += 0.05
            self.node.tentacle.actuator.cable.findData('value').value = displacement
            print("displacement = ", displacement)
