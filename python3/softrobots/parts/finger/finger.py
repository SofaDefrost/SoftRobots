# -*- coding: utf-8 -*-
import Sofa
import Sofa.Core
from Sofa.constants import *
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from stlib3.scene import Node
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh

from splib3.loaders import loadPointListFromFile
import os.path

templatepath = os.path.abspath(os.path.dirname(__file__) )

class FingerController(Sofa.Core.Controller):
    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        return

    def onKeypressedEvent(self,e):
        inputvalue = self.node.aCableActuator.value

        displacement = 0
        if (e["key"] == Sofa.constants.Key.plus):
            displacement = inputvalue.value[0] + 1.0
        elif (e["key"] == Sofa.constants.Key.minus):
            displacement = inputvalue.value[0] - 1.0
            if(displacement < 0):
                displacement = 0

        inputvalue.value = [displacement]
        return

def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[0.0,0.0,0.0], pullPointLocation=[0.0,0.0,0.0], youngModulus=18000, valueType='position'):

    eobject = ElasticMaterialObject(name=name, volumeMeshFileName=os.path.join(templatepath, "mesh/finger.vtk"), #MISK need to change the relative file
                                   poissonRatio=0.3,
                                   youngModulus=youngModulus,
                                   totalMass=0.5,
                                   surfaceColor=[0.0, 0.8, 0.65],
                                   surfaceMeshFileName=os.path.join(templatepath, "mesh/finger.stl"),
                                   rotation=rotation,
                                   translation=translation)
    parentNode.addChild(eobject)


    FixedBox(eobject, atPositions=fixingBox, doVisualization=True)

    cable=PullingCable(eobject,
                 "PullingCable",
                 pullPointLocation=pullPointLocation,
                 rotation=rotation,
                 translation=translation,
                 cableGeometry=loadPointListFromFile(os.path.join(templatepath, "mesh/cable.json")),
                 valueType=valueType);

    #Eulalie.C (21/09/18): this feature does not work, either fix it or remove this comment before SoftRobots v19
    #FingerController(eobject, cable, valueType) #MISK may change to vary variation based on value type

    CollisionMesh(eobject, name="CollisionMesh",
                 surfaceMeshFileName=os.path.join(templatepath, "mesh/finger.stl"),
                 rotation=rotation, translation=translation,
                 collisionGroup=[1, 2])

    CollisionMesh(eobject, name="CollisionMeshAuto1",
                 surfaceMeshFileName=os.path.join(templatepath, "mesh/fingerCollision_part1.stl"),
                 rotation=rotation, translation=translation,
                 collisionGroup=[1])

    CollisionMesh(eobject, name="CollisionMeshAuto2",
                 surfaceMeshFileName=os.path.join(templatepath, "mesh/fingerCollision_part2.stl"),
                 rotation=rotation, translation=translation,
                 collisionGroup=[2])


    return finger

def createScene(rootNode):
    from stlib.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)

    Finger(rootNode, translation=[1.0,0.0,0.0])
    return rootNode
