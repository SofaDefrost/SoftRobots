# -*- coding: utf-8 -*-

import Sofa

import os
from TentacleController import TentacleController
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'


def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', pluginName=[
                                "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                                "Sofa.Component.Collision.Detection.Algorithm",
                                # Needed to use components BVHNarrowPhase, BruteForceBroadPhase, DefaultPipeline
                                "Sofa.Component.Collision.Detection.Intersection",  # Needed to use components LocalMinDistance
                                "Sofa.Component.Collision.Response.Contact",  # Needed to use components DefaultContactManager
                                "Sofa.Component.Constraint.Lagrangian.Correction",
                                # Needed to use components LinearSolverConstraintCorrection
                                "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                                "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                                "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader, MeshVTKLoader
                                "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                                "Sofa.Component.Mass",  # Needed to use components UniformMass
                                "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                                "Sofa.Component.Setting",  # Needed to use components BackgroundSetting
                                "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                                "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                                "Sofa.Component.Topology.Container.Dynamic",
                                # Needed to use components TetrahedronSetTopologyContainer, TetrahedronSetTopologyModifier
                                "Sofa.Component.Visual",  # Needed to use components VisualStyle
                                "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel, OglSceneFrame
                            ])
    rootNode.addObject('VisualStyle', displayFlags="showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe")
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-3)

    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase', name="N2")
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response="FrictionContactConstraint", responseParams="mu=0")
    rootNode.addObject('LocalMinDistance', name="Proximity", alarmDistance=1, contactDistance=0.1)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    rootNode.findData('gravity').value=[0, 0, -9810]

    rootNode.addObject(TentacleController(node=rootNode))

    # FEM Model
    tentacle = rootNode.addChild('tentacle')
    tentacle.addObject('EulerImplicitSolver', name='odesolver', rayleighMass=0.1, rayleighStiffness=0.1)
    tentacle.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')

    tentacle.addObject('MeshVTKLoader', name='loader', filename=path+'Tentacle.vtk')
    tentacle.addObject('TetrahedronSetTopologyContainer', src='@loader')
    tentacle.addObject('TetrahedronSetTopologyModifier')

    tentacle.addObject('MechanicalObject', name='tetras', template='Vec3')
    tentacle.addObject('UniformMass', totalMass=0.024)
    tentacle.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=200)

    tentacle.addObject('BoxROI', name='ROI1', box=[[50, -20, 30],[75, 20, 50]], drawBoxes=True)
    tentacle.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)

    tentacle.addObject('LinearSolverConstraintCorrection')

    # Actuator
    actuator = tentacle.addChild('actuator')
    actuator.addObject('MechanicalObject', name="actuatorPoint",
                        position=[[48.2737, 1.44329e-15, 24.5372],
                                        [66.9662, -2.10942e-15, 13.5481],
                                        [70.5244, -7.77156e-16, 11.8775],
                                        [82.0908, -8.32667e-17, 6.00442],
                                        [84.7668, 0, 3.42029],
                                        [96.343, 4.44089e-16, -4.22773],
                                        [99.9109, -3.60822e-16, -7.67323],
                                        [111.516, 3.81639e-17, -20.6461],
                                        [114.192, 2.22045e-16, -23.2302],
                                        [124.024, 0, -36.2552],
                                        [125.817, 6.55725e-16, -39.7529],
                                        [133.884, 1.73472e-16, -54.605],
                                        [135.678, -4.30211e-16, -58.1027],
                                        [142.853, -5.55112e-17, -72.0935],
                                        [143.755, 0, -74.7298],
                                        [158.056, -2.498e-16, -93.8366],
                                        [158.056, -2.498e-16, -95]])
    actuator.addObject('CableConstraint', template='Vec3',
                        name="cable",
                        indices=list(range(15)),
                        pullPoint=[40, 0, 2],
                        valueType='displacement'
                        )
    actuator.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    # Visualization
    tentacleVisu = tentacle.addChild('visu')
    tentacleVisu.addObject('MeshSTLLoader', filename=path+"Tentacle.stl", name="loader")
    tentacleVisu.addObject('OglModel', src="@loader", color=[0.0, 0.7, 0.7])
    tentacleVisu.addObject('BarycentricMapping')

    # Contact
    tentacleContact = tentacle.addChild('contact')
    tentacleContact.addObject('MechanicalObject',
                                position=[
                                         [64,    0,   11], [69,  7,     8], [69,  -7,     8],   [71,  0,    17],
                                         [107,   0,  -23], [111, 7,   -27], [111, -7,   -27],   [117, 0,    -17],
                                         [93,    0, -7.5], [97,  7,   -11], [97,  -7,   -11],   [102, 0,    -0.5],
                                         [138,   0,  -73], [141, 7,   -77], [141, -7,   -77],   [146, 0,    -72],
                                         [78,    0,    3], [83,  7,     0], [83,  -7,     0],   [86,  0,    9],
                                         [118,   0,  -38], [122, 6.7, -42], [122, -7,    -42],  [129, -0.2, -35 ],
                                         [129.5, 0,-55.5], [132, 7,   -60], [132.5,-7, -59.6],  [138, 0,    -53.5]])
    tentacleContact.addObject('UnilateralPlaneConstraint', name='upc1', indices=[0 ,1, 2, 3])
    tentacleContact.addObject('UnilateralPlaneConstraint', name='upc2', indices=[4 ,5, 6, 7])
    tentacleContact.addObject('UnilateralPlaneConstraint', name='upc3', indices=[8 ,9, 10, 11])
    tentacleContact.addObject('UnilateralPlaneConstraint', name='upc4', indices=[12, 13, 14, 15])
    tentacleContact.addObject('UnilateralPlaneConstraint', name='upc5', indices=[16, 17, 18, 19])
    tentacleContact.addObject('UnilateralPlaneConstraint', name='upc6', indices=[20, 21, 22, 23])
    tentacleContact.addObject('UnilateralPlaneConstraint', name='upc7', indices=[24, 25, 26, 27])
    tentacleContact.addObject('BarycentricMapping')

    return rootNode
