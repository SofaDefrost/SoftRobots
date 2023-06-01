import Sofa

import os
from ControllerVolume import ControllerVolume
path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=100, tolerance=0.0000001)

    # bunny mechanical model
    bunny = rootNode.addChild('bunny')
    bunny.addObject('EulerImplicitSolver', name='odesolver')
    bunny.addObject('SparseLDLSolver', name='LDLsolver')
    bunny.addObject('MeshVTKLoader', name='loader', filename=path + 'Hollow_Stanford_Bunny.vtu')
    bunny.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    bunny.addObject('TetrahedronSetTopologyModifier')

    bunny.addObject('MechanicalObject', name='tetras', template='Vec3')
    bunny.addObject('UniformMass', totalMass=0.5)
    bunny.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                    youngModulus=18000)

    bunny.addObject('BoxROI', name='boxROI', box=[-5, -15, -5, 5, -4.5, 5], drawBoxes=True,
                    position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
    bunny.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
    bunny.addObject('LinearSolverConstraintCorrection')

    # bunny/cavity
    cavity = bunny.addChild('cavity')
    cavity.addObject('MeshOBJLoader', name='loader', filename=path + 'Hollow_Bunny_Body_Cavity.obj')
    cavity.addObject('MeshTopology', src='@loader', name='topo')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureConstraint', name="surfaceConstraint", triangles='@topo.triangles',
                     valueType="volumeGrowth")
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    bunny.addObject(ControllerVolume(node=bunny))

    # bunny/bunnyVisu
    bunnyVisu = bunny.addChild('visu')
    bunnyVisu.addObject('TriangleSetTopologyContainer', name='container')
    bunnyVisu.addObject('TriangleSetTopologyModifier')
    bunnyVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container", output="@container")

    bunnyVisu.addObject('OglModel', color=[0.3, 0.2, 0.2, 0.6])
    bunnyVisu.addObject('IdentityMapping')

    return rootNode
