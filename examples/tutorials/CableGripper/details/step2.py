# -*- coding: utf-8 -*-
from stlib3.physics.deformable import ElasticMaterialObject


def Finger(parentNode=None, name="Finger"):
    finger = parentNode.addChild(name)
    mobject = ElasticMaterialObject(finger, volumeMeshFileName="data/mesh/finger.vtk")
    finger.addChild(mobject)

    return None


def createScene(rootNode):
    # -*- coding: utf-8 -*-
    from stlib3.scene import MainHeader
    m = MainHeader(rootNode, plugins=["SoftRobots"])
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    m.getObject("VisualStyle").displayFlags = 'showForceFields showBehaviorModels showInteractionForceFields'

    Finger(rootNode)

    return rootNode
