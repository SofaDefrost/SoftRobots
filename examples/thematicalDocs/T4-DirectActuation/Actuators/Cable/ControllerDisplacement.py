#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core
import Sofa.constants.Key as Key


class ControllerDisplacement(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.name='ControllerDisplacement'
        self.node = kw["node"]
        return

    def onKeypressedEvent(self, e):
        inputvalue = self.node.aCable.value

        displacement = 0
        if e["key"] == Key.plus:
            displacement = inputvalue.value[0] + 1.0
        elif e["key"] == Key.minus:
            displacement = inputvalue.value[0] - 1.0
            if displacement < 0:
                displacement = 0

        inputvalue.value = [displacement]
        return
