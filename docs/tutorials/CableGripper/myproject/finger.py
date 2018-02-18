# -*- coding: utf-8 -*-
import Sofa
from stlib.scene import Node
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox
from stlib.physics.collision import CollisionMesh
from stlib.tools import loadPointListFromFile
from softrobots.actuators import PullingCable

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

def Finger(parentNode=None, Name="Finger",
           rotation=[0.0, 0.0, 0.0], translate=[0.0, 0.0, 0.0],
           withFixingBox=[0.0,0.0,0.0], withPullPointLocation=[0.0,0.0,0.0]):

    finger = Node(parentNode, Name)

    eobject = ElasticMaterialObject(
                                   finger,
                                   fromVolumeMesh="data/mesh/finger.vtk",
                                   withPoissonRatio=0.3,
                                   withYoungModulus=18000,
                                   withTotalMass=0.5,
                                   withSurfaceMesh="data/mesh/finger.stl",
                                   rotation=rotation,
                                   withSurfaceColor=[0.0,0.7,0.8],
                                   Name=Name)

    FixedBox(applyTo=eobject,
                                atPositions=withFixingBox,
                                withVisualization=True)

    cable=PullingCable(eobject,
                 Name="PullingCable",
                 withAPullPointLocation=withPullPointLocation,
                 rotation=rotation,
                 Name=Name,
                 withCableGeometry=loadPointListFromFile("data/mesh/cablepoints.json"));

    FingerController(eobject, cable)

    CollisionMesh(parentNode=eobject,
                 fromSurfaceMesh="data/mesh/finger.stl",
                 rotation=rotation, Name=Name,
                 Name="CollisionMesh", withACollisionGroup=[1, 2])

    CollisionMesh(parentNode=eobject,
                 fromSurfaceMesh="data/mesh/fingerCollision_part1.stl",
                 rotation=rotation, Name=Name,
                 Name="CollisionMeshAuto1", withACollisionGroup=[1])

    CollisionMesh(parentNode=eobject,
                 fromSurfaceMesh="data/mesh/fingerCollision_part2.stl",
                 rotation=rotation, Name=Name,
                 Name="CollisionMeshAuto2", withACollisionGroup=[2])

    return finger

def createScene(rootNode):
    from stlib.scene import MainHeader
    from stlib.visuals import ShowGrid
    from stlib.physics.rigid import Floor
    from stlib.physics.rigid import Cube
    m=MainHeader(rootNode, plugins=["SoftRobots"])
    m.getObject("VisualStyle").displayFlags='showForceFields showBehaviorModels showInteractionForceFields'

    ## Creation de modèle 
    ShowGrid(rootNode)
    Finger(rootNode, withFixingBox=[-10,-10,-5,10,10,15], withPullPointLocation=[0.0,10,0.0])
   
    return rootNode
