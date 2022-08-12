from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
# -*- coding: utf-8 -*-
from softrobots.actuators import PullingCable
from splib3.loaders import loadPointListFromFile
import Sofa.Core
import Sofa.constants.Key as Key


class FingerController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.plus:
            displacement += 1.

        elif e["key"] == Key.minus:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]


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
    cable = PullingCable(eobject, cableGeometry=loadPointListFromFile("data/mesh/cable.json"))

    finger.addObject(FingerController(cable))


def createScene(rootNode):
    # -*- coding: utf-8 -*-
    from stlib3.scene import MainHeader
    MainHeader(rootNode, plugins=["SoftRobots"])
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=100)

    Finger(rootNode)

    return rootNode
