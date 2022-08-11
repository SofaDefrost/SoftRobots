# -*- coding: utf-8 -*-
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox


def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-10, -10, -10, 10, 10, 10]):
    finger = parentNode.addChild(name)
    eobject = ElasticMaterialObject(finger,
                                    volumeMeshFileName="data/mesh/finger.vtk",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=0.5,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName="data/mesh/finger.stl",
                                    rotation=rotation,
                                    translation=translation)
    finger.addChild(eobject)

    FixedBox(eobject,
             doVisualization=True,
             atPositions=fixingBox)


def createScene(rootNode):
    # -*- coding: utf-8 -*-
    from stlib3.scene import MainHeader
    MainHeader(rootNode, plugins=["SoftRobots"])
    rootNode.VisualStyle.displayFlags = 'showForceFields showBehaviorModels showInteractionForceFields'

    Finger(rootNode)

    return rootNode
