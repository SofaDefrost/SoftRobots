# -*- coding: utf-8 -*-
from finger import Finger

def Gripper(parentNode):
    """A three finger soft gripper."""
    gripper = Node(parentNode, "Gripper")

    f1 = Finger(gripper, 
           Name="Finger1",
           rotation=[0, 0, 105],
           translate=[20.0,0.0, 0.0],
           withFixingBox=[-20, -10, 0, 20, 10, 15],
           withPullPointLocation=[3, 10.5, 3])

    return gripper


def createScene(rootNode):
    """To test the content of this file, try:    
    >>> runSofa gripper.py 
    """ 
    from stlib.scene import MainHeader, ContactHeader
    m=MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    m.getObject("VisualStyle").displayFlags='showForceFields showBehaviorModels showInteractionForceFields'
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Gripper(rootNode)
    return rootNode


