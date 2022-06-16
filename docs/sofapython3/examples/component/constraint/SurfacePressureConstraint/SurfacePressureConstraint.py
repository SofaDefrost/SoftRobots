import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', pluginName=[
                                "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                                "Sofa.Component.Constraint.Lagrangian.Correction",
                                # Needed to use components LinearSolverConstraintCorrection
                                "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                                "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                                "Sofa.Component.IO.Mesh",  # Needed to use components MeshOBJLoader, MeshVTKLoader
                                "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                                "Sofa.Component.LinearSolver.Iterative",  # Needed to use components ShewchukPCGLinearSolver
                                "Sofa.Component.Mass",  # Needed to use components UniformMass
                                "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                                "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                                "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                                "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                                "Sofa.Component.Topology.Container.Dynamic",
                                # Needed to use components TetrahedronSetTopologyContainer, TetrahedronSetTopologyModifier, TriangleSetTopologyContainer, TriangleSetTopologyModifier
                                "Sofa.Component.Topology.Mapping",  # Needed to use components Tetra2TriangleTopologicalMapping
                                "Sofa.Component.Visual",  # Needed to use components VisualStyle
                                "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
                            ])
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance=0.0000001)

    # bunny
    bunny = rootNode.addChild('bunny')
    bunny.addObject('EulerImplicitSolver', name='odesolver')
    bunny.addObject('ShewchukPCGLinearSolver', iterations=15, name='linearsolver', tolerance=1e-5,
                    preconditioners='preconditioner', use_precond=True, update_step=1)

    bunny.addObject('MeshVTKLoader', name='loader', filename=path + 'Hollow_Stanford_Bunny.vtu')
    bunny.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    bunny.addObject('TetrahedronSetTopologyModifier')

    bunny.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False)
    bunny.addObject('UniformMass', totalMass=0.5)
    bunny.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                    youngModulus=18000)

    bunny.addObject('BoxROI', name='boxROI', box=[-5, -15, -5, 5, -4.5, 5], drawBoxes=True,
                    position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
    bunny.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)

    bunny.addObject('SparseLDLSolver', name='preconditioner')
    bunny.addObject('LinearSolverConstraintCorrection', solverName='preconditioner')

    # bunny/cavity
    cavity = bunny.addChild('cavity')
    cavity.addObject('MeshOBJLoader', name='loader', filename=path + 'Hollow_Bunny_Body_Cavity.obj')
    cavity.addObject('MeshTopology', src='@loader', name='topo')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureConstraint', triangles='@topo.triangles', value=40, valueType=1)
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    # bunny/bunnyVisu
    bunnyVisu = bunny.addChild('visu')
    bunnyVisu.addObject('TriangleSetTopologyContainer', name='container')
    bunnyVisu.addObject('TriangleSetTopologyModifier')
    bunnyVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container", output="@container")

    bunnyVisu.addObject('OglModel', color=[0.3, 0.2, 0.2, 0.6])
    bunnyVisu.addObject('IdentityMapping')

    return rootNode
