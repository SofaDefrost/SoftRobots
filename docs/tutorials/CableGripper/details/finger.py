import Sofa
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox as FixedBoxConstraint

from softrobots.actuators import PullingCable
from stlib.physics.collision import CollisionMesh

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

    finger = ElasticMaterialObject(fromVolumeMesh="data/mesh/finger.vtk",
                                   Name=Name,
                                   withPoissonRatio=0.3,
                                   withYoungModulus=18000,
                                   withTotalMass=0.5,
                                   withSurfaceColor=[0.0, 0.8, 0.7],
                                   withSurfaceMesh="data/mesh/finger.stl",
                                   rotation=rotation,
                                   Name=Name,
                                   parentNode=parentNode)

    FixedBoxConstraint(atPositions=withFixingBox, applyTo=finger,
                       withVisualization=True)

    cable=PullingCable(parentNode=finger,
                 Name="PullingCable",
                 withAPullPointLocation=withPullPointLocation,
                 rotation=rotation,
                 Name=Name,
                 withCableGeometry=[[-17.5, 12.5, 2.5],
                                    [-32.5, 12.5, 2.5],
                                    [-47.5, 12.5, 2.5],
                                    [-62.5, 12.5, 2.5],
                                    [-77.5, 12.5, 2.5],
                                    [-83.5, 12.5, 4.5],
                                    [-85.5, 12.5, 6.5],
                                    [-85.5, 12.5, 8.5],
                                    [-83.5, 12.5, 10.5],
                                    [-77.5, 12.5, 12.5],
                                    [-62.5, 12.5, 12.5],
                                    [-47.5, 12.5, 12.5],
                                    [-32.5, 12.5, 12.5],
                                    [-17.5, 12.5, 12.5]]);

    FingerController(finger, cable)

    CollisionMesh(parentNode=finger,
                 fromSurfaceMesh="data/mesh/finger.stl",
                 rotation=rotation, Name=Name,
                 Name="CollisionMesh", withACollisionGroup=[1, 2])

    CollisionMesh(parentNode=finger,
                 fromSurfaceMesh="data/mesh/fingerCollision_part1.stl",
                 rotation=rotation, Name=Name,
                 Name="CollisionMeshAuto1", withACollisionGroup=[1])

    CollisionMesh(parentNode=finger,
                 fromSurfaceMesh="data/mesh/fingerCollision_part2.stl",
                 rotation=rotation, Name=Name,
                 Name="CollisionMeshAuto2", withACollisionGroup=[2])


    return finger

def createScene(rootNode):
    from stlib.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Finger(rootNode, translate=[1.0,0.0,0.0])
    return rootNode

