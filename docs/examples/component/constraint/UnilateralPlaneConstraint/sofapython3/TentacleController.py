#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import Sofa.Core

class FingerController(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        return

    def reset(self):

        displacement = 0
	    self.node.getChild('tentacle').getChild('actuator').getObject('cable').findData('value').value = str(displacement)


    def onAnimateBeginEvent(self, eventType):

        displacement = self.node.getChild('tentacle').getChild('actuator').getObject('cable').findData('value').value[0][0]

        if(displacement < 35):
            displacement += 0.05
            self.node.getChild('tentacle').getChild('actuator').getObject('cable').findData('value').value = str(displacement)
            print "displacement = "+str(displacement)
