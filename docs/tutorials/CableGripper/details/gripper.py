# -*- coding: utf-8 -*-
import Sofa
from grippercontroller import GripperController
from finger import Finger


def Gripper(parentNode=None):
    selfNode = parentNode.createChild("Gripper")

    f1 = Finger(selfNode, "Finger1",
           withRotation=[0, 0, 105],
           withTranslation=[20.0,0.0, 0.0],
           withFixingBox=[-20, -10, 0, 20, 10, 15],
           withPullPointLocation=[3, 10.5, 3])

    f2 = Finger(selfNode, "Finger2",
           withRotation=[180, 0, 65],
           withTranslation=[-10.0,0.0, -4.0],
           withFixingBox=[-20, -10, -30, 20, 10, 0],
           withPullPointLocation=[3, 10.5, -7])

    f3 = Finger(selfNode, "Finger3",
           withRotation=[180, 0, 65],
           withTranslation=[-10.0,0.0, 34.0],
           withFixingBox=[-20, -10, 0, 20, 10, 50],
           withPullPointLocation=[3, 10.5, 31.5])

    GripperController(selfNode, [f1,f2,f3])

    return selfNode


def createScene(rootNode):
    from stlib.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Gripper(parentNode=rootNode)
    return rootNode


