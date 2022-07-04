# -*- coding: utf-8 -*-
import Sofa.Core
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox

from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile


####################################################################################################
# FingerController
####################################################################################################
class FingerController(Sofa.Core.Controller):
    def __init__(self, *args, **kw):
        Sofa.Core.Controller.__init__(self,args,kw)
        cable = args[0]
        self.cableconstraintvalue = cable.getObject("CableConstraint").findData('value')
        self.name = "FingerController"

    def onKeyPressedEvent(self, c):
        print("Key pressed " + str(c))


####################################################################################################
# Finger
####################################################################################################
def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[0.0, 0.0, 0.0], pullPointLocation=[0.0, 0.0, 0.0]):
    finger = parentNode.addChild(name)

    # YOU NEED TO PUT THE FINGER CONTENT HERE.

    return finger


####################################################################################################
# You can test the gripper by typing runSofa finger.py
####################################################################################################
def createScene(rootNode):
    from stlib3.scene import MainHeader, ContactHeader

    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    Finger(rootNode, translation=[1.0,0.0,0.0])
    return rootNode
