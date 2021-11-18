# -*- coding: utf-8 -*-
from modules.accordion3 import addAccordion


def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('VisualStyle', displayFlags="showVisualModels hideBehaviorModels hideCollisionModels \
                            hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe")

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-14)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.gravity.value=[0, 0, -981.0]
    rootNode.dt.value=0.01

    # FEM Model
    accordion = addAccordion(rootNode)

    # For local communication
    communication = accordion.addObject('CommunicationController', name="sub", listening=True, job="receiver", port=5558, nbDataField=4, pattern=0)
    # Between two different computers, specify the ip address of the sender
    # accordion.createObject('CommunicationController', name="sub", listening=True, job="receiver", port=5558, nbDataField=4, ip="...")

    accordion.cavity.pressure.value.value=communication.data1.getLinkPath()
    for i in range(3):
        accordion.cables.getObject('cable'+str(i+1)).findData('value').value=communication.findData('data'+str(i+2)).getLinkPath()

    return rootNode
