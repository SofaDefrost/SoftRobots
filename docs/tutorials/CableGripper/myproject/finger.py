# -*- coding: utf-8 -*-
import Sofa
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox

from softrobots.actuators import PullingCable
from stlib.physics.collision import CollisionMesh
from stlib.tools import loadPointListFromFile
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
           withRotation=[0.0, 0.0, 0.0], withTranslation=[0.0, 0.0, 0.0],
           withFixingBox=[0.0,0.0,0.0], withPullPointLocation=[0.0,0.0,0.0]):

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
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Finger(rootNode,
           withTranslation=[1.0,0.0,0.0])
    return rootNode

