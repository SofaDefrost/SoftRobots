import os
from stlib3.physics.deformable import ElasticMaterialObject
from softrobots.actuators import PneumaticCavity
from stlib3.physics.constraints import FixedBox

meshpath = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def Bunny(node, translation=[0, 0, 0], controlType='pressure', name='Bunny', initialValue=0.0001,
          youngModulus=18000):
    # Bunny
    # boxROICoordinates=[-5, -6, -5,  5, -4.5, 5] + [translation,translation]
    boxROICoordinates = [-5 + translation[0], -6 + translation[1], -5 + translation[2], 5 + translation[0],
                         -4.5 + translation[1], 5 + translation[2]]
    Bunny = ElasticMaterialObject(name=name,
                                  volumeMeshFileName=meshpath + 'Hollow_Stanford_Bunny.vtu',
                                  surfaceMeshFileName=meshpath + 'Hollow_Bunny_Body_Cavity.obj',
                                  youngModulus=youngModulus,
                                  withConstrain=True,
                                  totalMass=0.5,
                                  color=[1, 1, 1, 1],
                                  translation=translation
                                  )
    node.addChild(Bunny)

    FixedBox(Bunny, doVisualization=True, atPositions=boxROICoordinates)

    if controlType == 'pressure':
        PneumaticCavity(name='Cavity', attachedAsAChildOf=Bunny,
                                 surfaceMeshFileName=meshpath + 'Hollow_Bunny_Body_Cavity.obj',
                                 valueType='pressure', initialValue=initialValue, translation=translation)
    elif controlType == 'volumeGrowth':
        PneumaticCavity(name='Cavity', attachedAsAChildOf=Bunny,
                                 surfaceMeshFileName=meshpath + 'Hollow_Bunny_Body_Cavity.obj',
                                 valueType='volumeGrowth', initialValue=initialValue, translation=translation)

    BunnyVisu = Bunny.addChild('visu')
    BunnyVisu.addObject('TriangleSetTopologyContainer', name='container')
    BunnyVisu.addObject('TriangleSetTopologyModifier')
    BunnyVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container", output="@container")
    BunnyVisu.addObject('OglModel', color=[0.3, 0.2, 0.2, 0.6], translation=translation)
    BunnyVisu.addObject('IdentityMapping')
    return Bunny


def createScene(rootNode):
    from stlib3.scene import MainHeader, ContactHeader
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots", 'SofaPython3'])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)

    rootNode.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.Constraint.Lagrangian.Correction",
                            # Needed to use components LinearSolverConstraintCorrection
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
                            "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
                            "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                            "Sofa.Component.Collision.Detection.Algorithm",
                            # Needed to use components BVHNarrowPhase, BruteForceBroadPhase, DefaultPipeline
                            "Sofa.Component.Collision.Detection.Intersection",  # Needed to use components LocalMinDistance
                            "Sofa.Component.Collision.Response.Contact",  # Needed to use components RuleBasedContactManager
                            "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                        ])

    Bunny(rootNode)
