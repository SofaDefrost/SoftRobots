#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core


class TentacleController(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        return

    def reset(self):
        displacement = 0
        self.node.tentacle.actuator.cable.findData('value').value = displacement

    def onAnimateBeginEvent(self, event):
        displacement = self.node.tentacle.actuator.cable.findData('value').value[0]

        if displacement < 35:
            displacement += 0.05
            self.node.tentacle.actuator.cable.findData('value').value = [displacement]
            print("displacement = ", displacement)
