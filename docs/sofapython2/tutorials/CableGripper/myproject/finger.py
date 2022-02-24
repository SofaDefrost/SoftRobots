# -*- coding: utf-8 -*-
import Sofa
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox

from softrobots.actuators import PullingCable
from stlib.physics.collision import CollisionMesh
from splib.loaders import loadPointListFromFile
from stlib.scene import Node

####################################################################################################
### FingerController
####################################################################################################
class FingerController(Sofa.PythonScriptController):
    def __init__(self, node, cable ):
        self.cableconstraintvalue = cable.getObject("CableConstraint").findData('value')
        self.name = "FingerController"

    def onKeyPressed(self,c):
        print("Key pressed "+str(c))

####################################################################################################
### Finger
####################################################################################################
def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[0.0,0.0,0.0], pullPointLocation=[0.0,0.0,0.0]):

    finger = Node(parentNode, name)

    #### YOU NEED TO PUT THE FINGER CONTENT HERE.

    return finger


####################################################################################################
### You can test the gripper by typing runSofa finger.py
####################################################################################################
def createScene(rootNode):
    """You can load the finger only by typing runSofa finger.py"""
    from stlib.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)

    Finger(rootNode,
           translation=[1.0,0.0,0.0])
    return rootNode

