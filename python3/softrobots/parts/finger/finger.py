# -*- coding: utf-8 -*-
import Sofa
import Sofa.Core
from Sofa.constants import *
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
import os.path
templatepath = os.path.abspath(os.path.dirname(__file__))


class FingerController(Sofa.Core.Controller):
    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        return

    def onKeypressedEvent(self, e):
        inputvalue = self.node.PullingCable.CableConstraint.value

        displacement = inputvalue.value
        if e["key"] == Sofa.constants.Key.plus:
            displacement = inputvalue.value[0] + 1.0
        elif e["key"] == Sofa.constants.Key.minus:
            displacement = inputvalue.value[0] - 1.0
            if displacement < 0:
                displacement = 0

        inputvalue.value = [displacement]
        return


def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-1.0, -1.0, -1.0, 1.0, 15.0, 15.0], pullPointLocation=[0.0, 0.0, 0.0], youngModulus=18000, valueType='position'):

    eobject = ElasticMaterialObject(name=name, volumeMeshFileName=os.path.join(templatepath, "mesh/finger.vtk"),
                                    # MISK need to change the relative file
                                    poissonRatio=0.3,
                                    youngModulus=youngModulus,
                                    totalMass=0.5,
                                    surfaceColor=[0.0, 0.8, 0.65, 1.0],
                                    surfaceMeshFileName=os.path.join(templatepath, "mesh/finger.stl"),
                                    rotation=rotation,
                                    translation=translation)
    parentNode.addChild(eobject)

    FixedBox(eobject, atPositions=fixingBox, doVisualization=True)

    PullingCable(eobject,
                 "PullingCable",
                 pullPointLocation=pullPointLocation,
                 rotation=rotation,
                 translation=translation,
                 cableGeometry=loadPointListFromFile(os.path.join(templatepath, "mesh/cable.json")),
                 valueType=valueType)

    eobject.addObject(FingerController(node=eobject))  # MISK may change to vary variation based on value type

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

    return eobject


def createScene(rootNode):
    from stlib3.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots", 'SofaPython3'])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)
    rootNode.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                            "Sofa.Component.Collision.Detection.Algorithm",
                            # Needed to use components BVHNarrowPhase, BruteForceBroadPhase, DefaultPipeline
                            "Sofa.Component.Collision.Detection.Intersection",  # Needed to use components LocalMinDistance
                            "Sofa.Component.Collision.Geometry",
                            # Needed to use components LineCollisionModel, PointCollisionModel, TriangleCollisionModel
                            "Sofa.Component.Collision.Response.Contact",  # Needed to use components RuleBasedContactManager
                            "Sofa.Component.Constraint.Lagrangian.Correction",
                            # Needed to use components LinearSolverConstraintCorrection
                            "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                            "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                            "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader, MeshVTKLoader
                            "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                            "Sofa.Component.Mass",  # Needed to use components UniformMass
                            "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                            "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                            "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                            "Sofa.Component.Topology.Container.Dynamic",  # Needed to use components TetrahedronSetTopologyContainer
                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                            "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel, OglSceneFrame
                        ])

    Finger(rootNode)
    return rootNode
