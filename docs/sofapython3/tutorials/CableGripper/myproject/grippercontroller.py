# -*- coding: utf-8 -*-
import Sofa.Core


def getTranslated(points, vec):
    r = []
    for v in points:
        r.append([v[0] + vec[0], v[1] + vec[1], v[2] + vec[2]])
    return r


class GripperController(Sofa.Core.Controller):
    def __init__(self, *args, **kw):
        Sofa.Core.Controller.__init__(self, args,kw)
        self.fingers = args[0]
        self.name = "FingerController"

    def onKeyPressed(self, c):
        direction = None
        # UP key :
        if ord(c) == 19:
            direction = [0.0, 1.0, 0.0]
        # DOWN key : rear
        elif ord(c) == 21:
            direction = [0.0, -1.0, 0.0]
        # LEFT key : left
        elif ord(c) == 18:
            direction = [1.0, 0.0, 0.0]
        elif ord(c) == 20:
            direction = [-1.0, 0.0, 0.0]

        if direction is not None:
            for finger in self.fingers:
                m = finger.getChild("ElasticMaterialObject")
                mecaobject = m.getObject("MechanicalObject")
                mecaobject.findData('rest_position').value = getTranslated(mecaobject.rest_position, direction)

                cable = m.getChild("PullingCable").getObject("CableConstraint")
                p = cable.pullPoint[0]
                cable.findData("pullPoint").value = [p[0] + direction[0], p[1] + direction[1], p[2] + direction[2]]
