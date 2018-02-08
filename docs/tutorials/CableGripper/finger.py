import Sofa
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox as FixedBoxConstraint

from softrobots.actuators import PullingCable
from stlib.physics.collision import CollisionMesh

class FingerController(Sofa.PythonScriptController):
    def __init__(self, cable ):
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

def Finger(attachedTo=None, withName="Finger",
           withRotation=[0.0, 0.0, 0.0], withTranslation=[0.0, 0.0, 0.0],
           withFixingBox=[0.0,0.0,0.0], withPullPointLocation=[0.0,0.0,0.0]):

    finger = ElasticMaterialObject(fromVolumeMesh="data/mesh/finger.vtk",
                                   withName=withName,
                                   withPoissonRatio=0.3,
                                   withYoungModulus=18000,
                                   withTotalMass=0.5,
                                   withSurfaceMesh="data/mesh/finger.stl",
                                   withRotation=withRotation,
                                   withTranslation=withTranslation,
                                   attachedTo=attachedTo)

    finger.createObject('ShewchukPCGLinearSolver', preconditioners='Solver',
                                                   update_step='1', name='linearsolver',
                                                   tolerance='1e-10', iterations='15',
                                                   use_first_precond='false', use_precond='true')


    FixedBoxConstraint(atPositions=withFixingBox, applyTo=finger,
                       withVisualization=True)

    cable=PullingCable(attachedTo=finger,
                 withName="PullingCable",
                 withAPullPointLocation=withPullPointLocation,
                 withRotation=withRotation,
                 withTranslation=withTranslation,
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

    finger.addObject( FingerController(cable) )

    CollisionMesh(attachedTo=finger,
                 fromSurfaceMesh="data/mesh/finger.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withName="CollisionMesh", withACollisionGroup=[1, 2])

    CollisionMesh(attachedTo=finger,
                 fromSurfaceMesh="data/mesh/fingerCollision_part1.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withName="CollisionMeshAuto1", withACollisionGroup=[1])

    CollisionMesh(attachedTo=finger,
                 fromSurfaceMesh="data/mesh/fingerCollision_part2.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withName="CollisionMeshAuto2", withACollisionGroup=[2])


    return finger
