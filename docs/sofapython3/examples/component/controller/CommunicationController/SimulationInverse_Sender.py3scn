# -*- coding: utf-8 -*-
from modules.accordion3 import addAccordion


def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('VisualStyle', displayFlags="showVisualModels hideBehaviorModels showCollisionModels \
                            hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe")

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', printLog=False, epsilon=1e-1, maxIterations=1000, tolerance=1e-14)
    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.gravity.value=[0, 0, -981.0]
    rootNode.dt.value=0.01

    accordion = addAccordion(rootNode, inverse=True)

    # Effector goal for interactive control
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalMO', position=[0, 0, 8])
    goal.addObject('SphereCollisionModel', radius=1)
    goal.addObject('UncoupledConstraintCorrection')

    effector = accordion.addChild('effector')
    effector.addObject('MechanicalObject', name="effectorPoint", position=[0, 0, 5])
    effector.addObject('PositionEffector', template='Vec3',
                       indices=0,
                       effectorGoal=goal.goalMO.position.getLinkPath(),
                       useDirections=[1, 1, 1])
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    accordion.cavity.pressure.minPressure=0
    accordion.cavity.pressure.maxVolumeGrowth=8
    for i in range(3):
        accordion.cables.getObject('cable'+str(i+1)).minForce=0
        accordion.cables.getObject('cable'+str(i+1)).maxPositiveDisp=1.5

    accordion.addObject('CommunicationController', listening=True, job="sender", port=5558, nbDataField=4, pattern=0,
                        data1="@cavity/pressure.pressure",
                        data2="@cables/cable1.displacement",
                        data3="@cables/cable2.displacement",
                        data4="@cables/cable3.displacement")

    return rootNode
