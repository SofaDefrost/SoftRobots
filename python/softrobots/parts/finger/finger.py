# -*- coding: utf-8 -*-
import Sofa
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox
from stlib.scene import Node
from softrobots.actuators import PullingCable
from stlib.physics.collision import CollisionMesh

from splib.loaders import loadPointListFromFile
import os.path

templatepath = os.path.abspath(os.path.dirname(__file__) )

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
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[0.0,0.0,0.0], pullPointLocation=[0.0,0.0,0.0], youngModulus=18000, valueType='position'):

    finger = Node(parentNode, name)
    eobject = ElasticMaterialObject(finger,
                                   volumeMeshFileName=os.path.join(templatepath, "mesh/finger.vtk"), #MISK need to change the relative file
                                   poissonRatio=0.3,
                                   youngModulus=youngModulus,
                                   totalMass=0.5,
                                   surfaceColor=[0.0, 0.8, 0.65],
                                   surfaceMeshFileName=os.path.join(templatepath, "mesh/finger.stl"),
                                   rotation=rotation,
                                   translation=translation)


    FixedBox(eobject, atPositions=fixingBox, doVisualization=True)

    cable=PullingCable(eobject,
                 "PullingCable",
                 pullPointLocation=pullPointLocation,
                 rotation=rotation,
                 translation=translation,
                 cableGeometry=loadPointListFromFile(os.path.join(templatepath, "mesh/cable.json")),
                 valueType=valueType);

    FingerController(eobject, cable, valueType) #MISK may change to vary variation based on value type

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
