# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key


def getTranslated(points, vec):
    r = []
    for v in points:
        r.append([v[0] + vec[0], v[1] + vec[1], v[2] + vec[2]])
    return r


class GripperController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.fingers = args[0]
        self.name = "GripperController"

    def onKeypressedEvent(self, e):
        direction = None

        if e["key"] == Key.uparrow:
            direction = [0.0, 1.0, 0.0]
        elif e["key"] == Key.downarrow:
            direction = [0.0, -1.0, 0.0]
        elif e["key"] == Key.leftarrow:
            direction = [1.0, 0.0, 0.0]
        elif e["key"] == Key.rightarrow:
            direction = [-1.0, 0.0, 0.0]

        if direction is not None and self.fingers is not None:
            for finger in self.fingers:
                m = finger.getChild("ElasticMaterialObject")
                mecaobject = m.getObject("dofs")
                mecaobject.findData('rest_position').value = getTranslated(mecaobject.rest_position.value, direction)

                cable = m.getChild("PullingCable").getObject("CableConstraint")
                p = cable.pullPoint.value
                cable.findData("pullPoint").value = [p[0] + direction[0], p[1] + direction[1], p[2] + direction[2]]


def createScene(rootNode):
    rootNode.addObject(GripperController(None))

    return
