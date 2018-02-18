# -*- coding: utf-8 -*-
import Sofa
from stlib.scene import Node
from grippercontroller import GripperController
from finger import Finger

####################################################################################################
### Gripper
####################################################################################################
def Gripper(parentNode=None):
    gripper = Node(parentNode, "Gripper")

    #### YOU NEED TO PUT THE GRIPPER CONTENT HERE.
    f1=Finger(gripper)
    f2=Finger(gripper)
    f3=Finger(gripper)

    GripperController(gripper, fingers=[f1,f2,f3])

    return gripper



####################################################################################################
### You can test the gripper by typing runSofa gripper.py
####################################################################################################
def createScene(rootNode):
    from stlib.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Gripper(rootNode)
    return rootNode


