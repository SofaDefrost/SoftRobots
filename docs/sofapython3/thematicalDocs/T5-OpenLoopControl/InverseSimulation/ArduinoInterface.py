#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core


class Controller(Sofa.Core.Controller):

    def __init__(self, *args, **kw):
        Sofa.Core.Controller.__init__(self,args,kw)
        self.name='Controller'
        self.node = args[0]
        self.leg1 = self.node.getChild('robot').getChild('leg1').getObject("sa")
        self.leg2 = self.node.getChild('robot').getChild('leg2').getObject("sa")
        self.leg3 = self.node.getChild('robot').getChild('leg3').getObject("sa")

        outputVector = [0., 0., 0.]
        self.node.getObject('serial').findData('sentData').value = outputVector

    def reset(self, node):
        outputVector = [0., 0., 0.]
        self.node.getObject('serial').findData('sentData').value = outputVector

    def onAnimateEndEvent(self, event):
        displacementLeg1 = self.leg1.findData("displacement").value
        displacementLeg2 = self.leg2.findData("displacement").value
        displacementLeg3 = self.leg3.findData("displacement").value
        outputVector = [displacementLeg1, displacementLeg2, displacementLeg3]

        self.node.getObject('serial').findData('sentData').value = outputVector
