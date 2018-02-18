# -*- coding: utf-8 -*-
import Sofa
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox

from softrobots.actuators import PullingCable
from stlib.physics.collision import CollisionMesh

from stlib.tools import loadPointListFromFile

class FingerController(Sofa.PythonScriptController):
    def __init__(self, node, cable ):
        self.cableconstraintvalue = cable.getObject("CableConstraint").findData('value')
        self.name = "FingerController"

    def onKeyPressed(self,c):
        if (c == "+"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] + 1.

        elif (c == "-"):
            displacement = self.cableconstraintvalue.value[0][0] - 1.
            if(displacement < 0):
                displacement = 0
            self.cableconstraintvalue.value = displacement

def Finger(parentNode=None, name="Finger",
           withRotation=[0.0, 0.0, 0.0], withTranslation=[0.0, 0.0, 0.0],
           withFixingBox=[0.0,0.0,0.0], withPullPointLocation=[0.0,0.0,0.0]):

    finger = ElasticMaterialObject(parentNode,
                                   fromVolumeMesh="data/mesh/finger.vtk",
                                   withName=name,
                                   withPoissonRatio=0.3,
                                   withYoungModulus=18000,
                                   withTotalMass=0.5,
                                   withSurfaceColor=[0.0, 0.8, 0.7],
                                   withSurfaceMesh="data/mesh/finger.stl",
                                   withRotation=withRotation,
                                   withTranslation=withTranslation)

    FixedBox(finger, atPositions=withFixingBox, withVisualization=True)

    cable=PullingCable(finger,
                 "PullingCable",
                 withAPullPointLocation=withPullPointLocation,
                 withRotation=withRotation,
                 withTranslation=withTranslation,
                 withCableGeometry=loadPointListFromFile("data/mesh/cable.json"));

    FingerController(finger, cable)

    CollisionMesh(finger, withName="CollisionMesh",
                 fromSurfaceMesh="data/mesh/finger.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withACollisionGroup=[1, 2])

    CollisionMesh(finger, withName="CollisionMeshAuto1",
                 fromSurfaceMesh="data/mesh/fingerCollision_part1.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withACollisionGroup=[1])

    CollisionMesh(finger, withName="CollisionMeshAuto2",
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

