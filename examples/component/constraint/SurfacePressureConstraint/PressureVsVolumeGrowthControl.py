import Sofa

from softrobots.parts.bunny import Bunny
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
                            "Sofa.Component.Mass",  # Needed to use components UniformMass
                            "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                            "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                            "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                            "Sofa.Component.Topology.Container.Dynamic",
                            # Needed to use components TetrahedronSetTopologyContainer, TriangleSetTopologyContainer, TriangleSetTopologyModifier
                            "Sofa.Component.Topology.Mapping",  # Needed to use components Tetra2TriangleTopologicalMapping
                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                            "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
                        ])
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels '
                                    'hideBoundingCollisionModels hideForceFields showInteractionForceFields '
                                    'hideWireframe')
    rootNode.dt = 0.001

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=500, printLog=False, tolerance=0.0000001)

    Bunny(rootNode, name='BunnyPressure', controlType='pressure', initialValue=2)
    Bunny(rootNode, translation=[15, 0, 0], controlType='volumeGrowth', name='BunnyVolume', initialValue=40)

    return rootNode
