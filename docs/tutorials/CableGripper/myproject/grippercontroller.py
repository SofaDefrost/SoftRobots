# -*- coding: utf-8 -*-
import Sofa

def getTranslated(points, vec):
    r=[]
    for v in points:
        r.append( [v[0]+vec[0], v[1]+vec[1], v[2]+vec[2]] )
    return r

class GripperController(Sofa.PythonScriptController):
    def __init__(self, node, fingers):
        self.fingers = fingers
        self.name = "FingerController"

    def onKeyPressed(self,c):
        dir = None
        # UP key :
        if ord(c)==19:
            dir = [0.0,1.0,0.0]
        # DOWN key : rear
        elif ord(c)==21:
            dir = [0.0,-1.0,0.0]
        # LEFT key : left
        elif ord(c)==18:
            dir = [1.0,0.0,0.0]
        elif ord(c)==20:
            dir = [-1.0,0.0,0.0]

        if dir != None:
            for finger in self.fingers:
                mecaobject = finger.getObject("MechanicalObject")
                mecaobject.findData('rest_position').value = getTranslated( mecaobject.rest_position,  dir )

                cable = finger.getChild("PullingCable").getObject("CableConstraint")
                p = cable.pullPoint[0]
                cable.findData("pullPoint").value = [p[0]+dir[0], p[1]+dir[1], p[2]+dir[2]]

