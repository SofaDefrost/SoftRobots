import os
from ControlKeyboard import ControlKeyboard

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


# units: cm and kg

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields ');

    rootNode.dt = 0.03
    rootNode.gravity = [0., 0., -9810]

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-6, maxIterations=50000)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    rootNode.addObject(ControlKeyboard(node=rootNode))

    ###############################
    # MECHANICAL MODEL
    ###############################

    robot = rootNode.addChild('robot')
    robot.addObject('EulerImplicitSolver', name='odesolver', firstOrder=False)
    robot.addObject('SparseLDLSolver')
    robot.addObject('MeshVTKLoader', name='loader', filename=path + 'branch.vtu')
    robot.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@loader.tetrahedra',
                    name='container', createTriangleArray=True)
    robot.addObject('TetrahedronSetTopologyModifier')

    robot.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5,
                    position='@loader.position')
    robot.addObject('UniformMass', totalMass=0.4)
    robot.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.45,
                    youngModulus=600)  # 600kPa

    robot.addObject('BoxROI', name='boxROI1', box=[-30, 80, 70, 30, 140, 130], drawBoxes=True)
    robot.addObject('PartialFixedProjectiveConstraint', name="pfc1", fixedDirections=[1, 1, 0], indices="@boxROI1.indices")
    robot.addObject('BoxROI', name='boxROI2', box=[-130, -100, 70, -60, -20, 130], drawBoxes=True)
    robot.addObject('PartialFixedProjectiveConstraint', name="pfc2", fixedDirections=[1, 1, 0], indices="@boxROI2.indices")
    robot.addObject('BoxROI', name='boxROI3', box=[30, -100, 70, 100, -20, 130], drawBoxes=True)
    robot.addObject('PartialFixedProjectiveConstraint', name="pfc3", fixedDirections=[1, 1, 0], indices="@boxROI3.indices")

    robot.addObject('LinearSolverConstraintCorrection')

    leg0 = rootNode.addChild('RestPositionLeg0')
    leg0.addObject('MechanicalObject', name='meca0', template='Vec3', showObject=True, showObjectScale=15,
                   showIndices=False, showIndicesScale=4e-5,
                   position='@../robot/boxROI1.pointsInROI')
    leg1 = rootNode.addChild('RestPositionLeg1')
    leg1.addObject('MechanicalObject', name='meca1', template='Vec3', showObject=True, showObjectScale=15,
                   showIndices=False, showIndicesScale=4e-5,
                   position='@../robot/boxROI2.pointsInROI')
    leg2 = rootNode.addChild('RestPositionLeg2')
    leg2.addObject('MechanicalObject', name='meca2', template='Vec3', showObject=True, showObjectScale=15,
                   showIndices=False, showIndicesScale=4e-5,
                   position='@../robot/boxROI3.pointsInROI')

    robot.addObject('RestShapeSpringsForceField', name='fixed1', points="@boxROI1.indices",
                    external_rest_shape="@RestPositionLeg0/meca0", stiffness=1e3)
    robot.addObject('RestShapeSpringsForceField', name='fixed2', points="@boxROI2.indices",
                    external_rest_shape="@RestPositionLeg1/meca1", stiffness=1e3)
    robot.addObject('RestShapeSpringsForceField', name='fixed3', points="@boxROI3.indices",
                    external_rest_shape="@RestPositionLeg2/meca2", stiffness=1e3)

    ###############################
    # VISUAL MODEL
    ###############################

    # Modele visuel
    visualrobot = robot.addChild('visualrobot')
    visualrobot.addObject('MeshSTLLoader', filename=path + 'branch.stl', name='loaderSurf')
    visualrobot.addObject('OglModel', name='mappedBodyVisual', src='@loaderSurf', color=[1., 1., 1., 1.])
    visualrobot.addObject('BarycentricMapping', name='BMVisual')

    return rootNode
