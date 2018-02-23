# -*- coding: utf-8 -*-
import Sofa
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox
from stlib.scene import Node
from softrobots.actuators import PullingCable
from stlib.physics.collision import CollisionMesh

from stlib.tools import loadPointListFromFile

class FingerController(Sofa.PythonScriptController):

    def __init__(self, node, cable, valueType ):
        self.cableconstraintvalue = cable.getObject("CableConstraint").findData('value')
        self.name = "FingerController"
        if(valueType == "position"):
            self.valueIncrement = 1
        else:
            self.valueIncrement = 20

    def onKeyPressed(self,c):
        if (c == "+"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] + self.valueIncrement

        elif (c == "-"):
            actuation = self.cableconstraintvalue.value[0][0] - self.valueIncrement
            if(actuation < 0):
                actuation = 0
            self.cableconstraintvalue.value = actuation

def Finger(parentNode=None, name="Finger",
           withRotation=[0.0, 0.0, 0.0], withTranslation=[0.0, 0.0, 0.0],
           withFixingBox=[0.0,0.0,0.0], withPullPointLocation=[0.0,0.0,0.0], withYoungModulus=18000, withCableValueAs='position'):

    finger = Node(parentNode, name)
    eobject = ElasticMaterialObject(finger,
                                   fromVolumeMesh="data/mesh/finger.vtk", #MISK need to change the relative file
                                   withPoissonRatio=0.3,
                                   withYoungModulus=withYoungModulus,
                                   withTotalMass=0.5,
                                   withSurfaceColor=[0.0, 0.8, 0.7],
                                   withSurfaceMesh="data/mesh/finger.stl",
                                   withRotation=withRotation,
                                   withTranslation=withTranslation)

    FixedBox(eobject, atPositions=withFixingBox, withVisualization=True)

    cable=PullingCable(eobject,
                 "PullingCable",
                 withAPullPointLocation=withPullPointLocation,
                 withRotation=withRotation,
                 withTranslation=withTranslation,
                 withCableGeometry=loadPointListFromFile("data/mesh/cable.json"),
                 withValueAs=withCableValueAs);

    FingerController(eobject, cable, withCableValueAs) #MISK may change to vary variation based on value type

    CollisionMesh(eobject, withName="CollisionMesh",
                 fromSurfaceMesh="data/mesh/finger.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withACollisionGroup=[1, 2])

    CollisionMesh(eobject, withName="CollisionMeshAuto1",
                 fromSurfaceMesh="data/mesh/fingerCollision_part1.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withACollisionGroup=[1])

    CollisionMesh(eobject, withName="CollisionMeshAuto2",
                 fromSurfaceMesh="data/mesh/fingerCollision_part2.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withACollisionGroup=[2])


    return finger

def createScene(rootNode):
    from stlib.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Finger(rootNode, withTranslate=[1.0,0.0,0.0])
    return rootNode
