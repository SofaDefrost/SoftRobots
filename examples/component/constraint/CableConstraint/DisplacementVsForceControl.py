# -*- coding: utf-8 -*-
import Sofa
from softrobots.parts.finger import Finger
from stlib3.scene import MainHeader, ContactHeader


def createScene(rootNode):
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

    # stiff, position controlled
    Finger(rootNode, name="StiffPosition",
                translation=[0.0, 0.0, 0.0], fixingBox=[-20, -10, 0, 20, 10, 15],
                youngModulus=18000, valueType='displacement', pullPointLocation=[0.0, 12.5, 2.5])
    # stiff, force controlled
    Finger(rootNode, name="StiffForce",
                translation=[150.0, 0.0, 0.0], fixingBox=[130, -10, 0, 170, 10, 15],
                youngModulus=18000, valueType='force', pullPointLocation=[150.0, 12.5, 2.5])
    # soft, position controlled
    Finger(rootNode, name="SoftPosition",
                translation=[0.0, 100.0, 0.0], fixingBox=[-20, 90, 0, 20, 110, 15],
                youngModulus=9000, valueType='displacement', pullPointLocation=[0.0, 112.5, 2.5])
    # soft, force controlled
    Finger(rootNode, name="SoftForce",
                translation=[150.0, 100.0, 0.0], fixingBox=[130, 90, 0, 170, 110, 15],
                youngModulus=9000, valueType='force', pullPointLocation=[150.0, 112.5, 2.5])
    return rootNode
