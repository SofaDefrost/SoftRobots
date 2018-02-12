import Sofa

from finger import Finger

def getTranslated(points, vec):
    r=[]
    for v in points:
        r.append( [v[0]+vec[0], v[1]+vec[1], v[2]+vec[2]] )
    return r

class GripperController(Sofa.PythonScriptController):
    def __init__(self, fingers):
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

def Gripper(attachedTo=None):
    selfNode = attachedTo.createChild("Gripper")

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

    selfNode.addObject(GripperController([f1,f2,f3]))

    return selfNode


def createScene(rootNode):
    from stlib.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Gripper(attachedTo=rootNode)
    return rootNode


