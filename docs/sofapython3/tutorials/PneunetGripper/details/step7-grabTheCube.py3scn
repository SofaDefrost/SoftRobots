import Sofa
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
                       pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver SofaMeshCollision SofaRigid SofaOpenglVisual')

    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    rootNode.gravity.value = [-9810, 0, 0];
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='FrictionContact', responseParams='mu=0.6')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=5, contactDistance=1, angleCone=0.0)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.])
    rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')

    planeNode = rootNode.addChild('Plane')
    planeNode.addObject('MeshObjLoader', name='loader', filename='data/mesh/floorFlat.obj', triangulate=True,
                        rotation=[0, 0, 270], scale=10, translation=[-122, 0, 0])
    planeNode.addObject('MeshTopology', src='@loader')
    planeNode.addObject('MechanicalObject', src='@loader')
    planeNode.addObject('TriangleCollisionModel', simulated=False, moving=False)
    planeNode.addObject('LineCollisionModel', simulated=False, moving=False)
    planeNode.addObject('PointCollisionModel', simulated=False, moving=False)
    planeNode.addObject('OglModel', name='Visual', src='@loader', color=[1, 0, 0, 1])

    cube = rootNode.addChild('cube')
    cube.addObject('EulerImplicitSolver', name='odesolver')
    cube.addObject('SparseLDLSolver', name='linearSolver')
    cube.addObject('MechanicalObject', template='Rigid3', position=[-100, 70, 0, 0, 0, 0, 1])
    cube.addObject('UniformMass', totalMass=0.001)
    cube.addObject('UncoupledConstraintCorrection')

    # collision
    cubeCollis = cube.addChild('cubeCollis')
    cubeCollis.addObject('MeshObjLoader', name='loader', filename='data/mesh/smCube27.obj', triangulate=True,
                         scale=6)
    cubeCollis.addObject('MeshTopology', src='@loader')
    cubeCollis.addObject('MechanicalObject')
    cubeCollis.addObject('TriangleCollisionModel')
    cubeCollis.addObject('LineCollisionModel')
    cubeCollis.addObject('PointCollisionModel')
    cubeCollis.addObject('RigidMapping')

    # visualization
    cubeVisu = cube.addChild('cubeVisu')
    cubeVisu.addObject('MeshObjLoader', name='loader', filename='data/mesh/smCube27.obj')
    cubeVisu.addObject('OglModel', name='Visual', src='@loader', color=[0.0, 0.1, 0.5], scale=6.2)
    cubeVisu.addObject('RigidMapping')

    for i in range(3):
        ##########################################
        # Finger Model	 						 #
        ##########################################
        finger = rootNode.addChild('finger' + str(i + 1))
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', name='preconditioner')

        finger.addObject('MeshVTKLoader', name='loader', filename='data/mesh/pneunetCutCoarse.vtk',
                         rotation=[360 - angles[i] * 180 / math.pi, 0, 0], translation=translations[i])
        finger.addObject('MeshTopology', src='@loader', name='container')

        finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)
        finger.addObject('UniformMass', totalMass=0.04)
        finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                         youngModulus=youngModulusFingers)

        finger.addObject('BoxROI', name='boxROI', box=[-10, 0, -20, 0, 30, 20], doUpdate=False)
        finger.addObject('BoxROI', name='boxROISubTopo', box=[-100, 22.5, -8, -19, 28, 8], strict=False)
        if i == 0:
            finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12,
                             angularStiffness=1e12)
        else:
            finger.addObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness=1e12,
                             angularStiffness=1e12)

        finger.addObject('LinearSolverConstraintCorrection', solverName='preconditioner')

        ##########################################
        # Sub topology						   #
        ##########################################
        modelSubTopo = finger.addChild('modelSubTopo')
        if i == 0:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position',
                                   tetrahedra='@boxROISubTopo.tetrahedraInROI', name='container')
        else:
            modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position',
                                   tetrahedra='@../../finger1/boxROISubTopo.tetrahedraInROI', name='container')
        modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large',
                               poissonRatio=0.3, youngModulus=youngModulusStiffLayerFingers - youngModulusFingers)

        ##########################################
        # Constraint							 #
        ##########################################
        cavity = finger.addChild('cavity')
        cavity.addObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCavityCut.stl',
                         translation=translations[i], rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
        cavity.addObject('MeshTopology', src='@loader', name='topo')
        cavity.addObject('MechanicalObject', name='cavity')
        cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=0.0001,
                         triangles='@topo.triangles', valueType='pressure')
        cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

        ##########################################
        # Collision							  #
        ##########################################

        collisionFinger = finger.addChild('collisionFinger')
        collisionFinger.addObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCut.stl',
                                  translation=translations[i], rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
        collisionFinger.addObject('MeshTopology', src='@loader', name='topo')
        collisionFinger.addObject('MechanicalObject', name='collisMech')
        collisionFinger.addObject('TriangleCollisionModel', selfCollision=False)
        collisionFinger.addObject('LineCollisionModel', selfCollision=False)
        collisionFinger.addObject('PointCollisionModel', selfCollision=False)
        collisionFinger.addObject('BarycentricMapping')

        ##########################################
        # Visualization						  #
        ##########################################
        modelVisu = finger.addChild('visu')
        modelVisu.addObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCut.stl')
        modelVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6], translation=translations[i],
                            rotation=[360 - angles[i] * 180 / math.pi, 0, 0])
        modelVisu.addObject('BarycentricMapping')

    rootNode.addObject(WholeGripperController(node=rootNode))

    return rootNode
