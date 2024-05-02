
import os
from ArduinoInterface import Controller

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


# The scene is in cm

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin',
                       pluginName=["SoftRobots", "SoftRobots.Inverse"])
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields ')

    rootNode.dt = 0.03
    rootNode.gravity = [0., 0., -9810]

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', epsilon=1e-3, printLog=0)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    # rootNode.addObject(Controller(rootNode))
    # rootNode.addObject('SerialPortBridgeGeneric', name="serial", port="/dev/ttyACM0", baudRate=115200, size=3, listening=True)

    ##########################################
    # EFFECTOR GOAL
    ##########################################
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=200, threshold=1e-5, tolerance=1e-5)
    goal.addObject('MechanicalObject', name='goalMO',
                   showObject=1,
                   showObjectScale=10,
                   drawMode=1,
                   position=[0, 0, 222])
    goal.addObject('UncoupledConstraintCorrection')

    ###############################
    # MECHANICAL MODEL
    ###############################

    robot = rootNode.addChild('robot')
    robot.addObject('EulerImplicitSolver', name='odesolver', firstOrder=False)
    robot.addObject('SparseLDLSolver')
    robot.addObject('MeshVTKLoader', name='loader', filename=path + 'branch.vtu')
    robot.addObject('MeshTopology', src='@loader')

    robot.addObject('MechanicalObject', name='tetras', template='Vec3', position='@loader.position')
    robot.addObject('UniformMass', totalMass=0.4)
    robot.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.45,
                    youngModulus=600)  # 600kPa

    robot.addObject('BoxROI', name='boxROI1', box=[-30, 80, 70, 30, 140, 130], drawBoxes=True)
    robot.addObject('PartialFixedProjectiveConstraint', name="pfc1", fixedDirections=[1, 1, 0], indices="@boxROI1.indices")
    robot.addObject('BoxROI', name='boxROI2', box=[-130, -100, 70, -60, -20, 130], drawBoxes=True)
    robot.addObject('PartialFixedProjectiveConstraint', name="pfc2", fixedDirections=[1, 1, 0], indices="@boxROI2.indices")
    robot.addObject('BoxROI', name='boxROI3', box=[130, -100, 70, 60, -20, 130], drawBoxes=True)
    robot.addObject('PartialFixedProjectiveConstraint', name="pfc3", fixedDirections=[1, 1, 0], indices="@boxROI3.indices")

    robot.addObject('LinearSolverConstraintCorrection')

    ###############################
    # EFFECTOR
    ###############################

    effector = robot.addChild('effector')
    effector.addObject('MechanicalObject', name="effectorPoint", position=[0, 0, 222])
    effector.addObject('PositionEffector', template='Vec3',
                       indices=0,
                       effectorGoal="@../../goal/goalMO.position")
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    ###############################
    # ACTUATORS
    ###############################

    maxDisp = 70
    minDisp = 20

    # Motors leg
    for i in range(1, 4):
        leg = robot.addChild('leg' + str(i))
        leg.addObject('MechanicalObject', position='@../boxROI' + str(i) + '.pointsInROI')
        leg.addObject('SlidingActuator', name="sa",
                      direction=[0, 0, 1], showDirection=1, showVisuScale=5,
                      maxPositiveDisp=maxDisp,
                      maxNegativeDisp=minDisp)
        leg.addObject('BarycentricMapping', mapMasses=False, mapForces=False)

    ###############################
    # VISUAL MODEL
    ###############################

    # Visual model
    visualrobot = robot.addChild('visualrobot')
    visualrobot.addObject('MeshSTLLoader', filename=path + 'branch.stl', name='loaderSurf')
    visualrobot.addObject('OglModel', name='mappedBodyVisual', src='@loaderSurf', color=[1., 1., 1., 1.])
    visualrobot.addObject('BarycentricMapping', name='BMVisual')

    # Visual model
    visualCoque = rootNode.addChild('coque')
    visualCoque.addObject('MeshSTLLoader', filename=path + 'coque_base.stl', name='loaderSurf',
                          translation=[0., 0., 10])
    visualCoque.addObject('OglModel', name='mappedBodyVisual', src='@loaderSurf', color=[0.2, 0.2, 0.2, 1.])

    return rootNode
