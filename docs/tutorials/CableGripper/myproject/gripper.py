from finger import Finger
from grippercontroller import GripperController

def Gripper(attachedTo=None):
    """A three finger soft gripper."""
    selfNode = attachedTo.createChild("Gripper")

    f1 = Finger(selfNode, "Finger1",
           withRotation=[0, 0, 105],
           withTranslation=[20.0,0.0, 0.0],
           withFixingBox=[-20, -10, 0, 20, 10, 15],
           withPullPointLocation=[3, 10.5, 3])

    return selfNode


def createScene(rootNode):
    """To test the content of this file, try:    
    >>> runSofa gripper.py 
    """ 
    from stlib.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Gripper(attachedTo=rootNode)
    return rootNode


