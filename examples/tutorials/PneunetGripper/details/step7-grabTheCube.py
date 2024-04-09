import math

from wholeGripperController import WholeGripperController

youngModulusFingers = 500
youngModulusStiffLayerFingers = 1500

radius = 70
angle1 = 120 * math.pi / 180  # Angle between 1st and 2nd finger in radian
angle2 = 240 * math.pi / 180  # Angle between 1st and 3rd finger in radian
translateFinger1 = [0, 0, 0]
translateFinger2 = [0, radius + radius * math.sin(angle1 - math.pi / 2), radius * math.cos(angle1 - math.pi / 2)]
translateFinger3 = [0, radius + radius * math.sin(angle2 - math.pi / 2), radius * math.cos(angle2 - math.pi / 2)]
translations = [translateFinger1, translateFinger2, translateFinger3]
angles = [0, angle1, angle2]


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin',
                       pluginName='SoftRobots SofaPython3')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')  # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')  # Needed to use components [LocalMinDistance]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')  # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')  # Needed to use components [CollisionResponse]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [GenericConstraintCorrection,UncoupledConstraintCorrection]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver')  # Needed to use components [GenericConstraintSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')  # Needed to use components [BoxROI]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')  # Needed to use components [MeshOBJLoader,MeshSTLLoader,MeshVTKLoader]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative')  # Needed to use components [CGLinearSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')  # Needed to use components [BarycentricMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')  # Needed to use components [RigidMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting')  # Needed to use components [BackgroundSetting]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring')  # Needed to use components [RestShapeSpringsForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')  # Needed to use components [TetrahedronSetTopologyContainer]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]  
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')  # Needed to use components [OglModel,OglSceneFrame]

    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels hideCollisionModels '
                                    'hideBoundingCollisionModels hideForceFields '
                                    'showInteractionForceFields hideWireframe')
    rootNode.gravity.value = [-9810, 0, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-7, maxIterations=1000)
    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('CollisionResponse', response='FrictionContactConstraint', responseParams='mu=0.6')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=5, contactDistance=1)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.])
    rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')

    ##########################################
    # Plane
    ##########################################
    plane = rootNode.addChild('Plane')
    plane.addObject('MeshOBJLoader', name='loader', filename='data/mesh/floorFlat.obj',
                    rotation=[0, 0, 270], scale=10, translation=[-122, 0, 0])
    plane.addObject('MeshTopology', src='@loader')
    plane.addObject('MechanicalObject', src='@loader')
    plane.addObject('TriangleCollisionModel')
    plane.addObject('LineCollisionModel')
    plane.addObject('PointCollisionModel')
    plane.addObject('OglModel', name='Visual', src='@loader', color=[1, 0, 0, 1])

    ##########################################
    # Cube
    ##########################################
    cube = rootNode.addChild('Cube')
    cube.addObject('EulerImplicitSolver')
    cube.addObject('CGLinearSolver', threshold=1e-5, tolerance=1e-5, iterations=50)
    cube.addObject('MechanicalObject', template='Rigid3', position=[-100, 70, 0, 0, 0, 0, 1])
    cube.addObject('UniformMass', totalMass=0.001)
    cube.addObject('UncoupledConstraintCorrection')

    # collision
    cubeCollis = cube.addChild('Collision')
    cubeCollis.addObject('MeshOBJLoader', name='loader', filename='data/mesh/smCube27.obj', scale=6)
    cubeCollis.addObject('MeshTopology', src='@loader')
    cubeCollis.addObject('MechanicalObject')
    cubeCollis.addObject('TriangleCollisionModel')
    cubeCollis.addObject('LineCollisionModel')
    cubeCollis.addObject('PointCollisionModel')
    cubeCollis.addObject('RigidMapping')

    # visualization
    cubeVisu = cube.addChild('Visu')
    cubeVisu.addObject('MeshOBJLoader', name='loader', filename='data/mesh/smCube27.obj')
    cubeVisu.addObject('OglModel', name='Visual', src='@loader', color=[0.0, 0.1, 0.5], scale=6.2)
    cubeVisu.addObject('RigidMapping')

    for i in range(3):
        ##########################################
        # Finger Model
        ##########################################
        finger = rootNode.addChild('Finger' + str(i + 1))
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
        finger.addObject('MeshVTKLoader', name='loader', filename='data/mesh/pneunetCutCoarse.vtk',
                         rotation=[360 - angles[i] * 180 / math.pi, 0, 0], translation=translations[i])
        finger.addObject('MeshTopology', src='@loader', name='container')
        finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
        finger.addObject('UniformMass', totalMass=0.04)
        finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                         youngModulus=youngModulusFingers)
        if i == 0:
            boxROI = finger.addObject('BoxROI', name='boxROI', box=[-10, 0, -20, 0, 30, 20], doUpdate=False)
            boxROISubTopo = finger.addObject('BoxROI', name='boxROISubTopo', box=[-100, 22.5, -8, -19, 28, 8], strict=False)
        finger.addObject('RestShapeSpringsForceField',
                         points=boxROI.indices.linkpath,
                         stiffness=1e12, angularStiffness=1e12)
        finger.addObject('GenericConstraintCorrection')

        # Sub topology
        modelSubTopo = finger.addChild('SubTopology')
        modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@../loader.position',
                               tetrahedra=boxROISubTopo.tetrahedraInROI.linkpath, name='container')
        modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large',
                               poissonRatio=0.3, youngModulus=youngModulusStiffLayerFingers - youngModulusFingers)

        # Constraint
        cavity = finger.addChild('Cavity')
        cavity.addObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCavityCut.stl',
                         translation=translations[i], rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
        cavity.addObject('MeshTopology', src='@loader', name='topo')
        cavity.addObject('MechanicalObject', name='cavity')
        cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3',
                         value=0.0001,
                         triangles='@topo.triangles', valueType='pressure')
        cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

        # Collision
        collisionFinger = finger.addChild('Collision')
        collisionFinger.addObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCut.stl',
                                  translation=translations[i], rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
        collisionFinger.addObject('MeshTopology', src='@loader', name='topo')
        collisionFinger.addObject('MechanicalObject')
        collisionFinger.addObject('TriangleCollisionModel')
        collisionFinger.addObject('LineCollisionModel')
        collisionFinger.addObject('PointCollisionModel')
        collisionFinger.addObject('BarycentricMapping')

        # Visualization
        modelVisu = finger.addChild('Visu')
        modelVisu.addObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCut.stl')
        modelVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6], translation=translations[i],
                            rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
        modelVisu.addObject('BarycentricMapping')

    rootNode.addObject(WholeGripperController(node=rootNode))

    return rootNode
