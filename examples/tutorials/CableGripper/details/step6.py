# -*- coding: utf-8 -*-
from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.rigid import Floor, Cube
from gripper import Gripper


def createScene(rootNode):
    """This is my first scene"""
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    Gripper(rootNode)

    Floor(rootNode,
          color=[1.0, 0.0, 0.0, 1.0],
          translation=[0.0, -160.0, 0.0],
          isAStaticObject=True)

    cube = Cube(rootNode,
                uniformScale=20.0,
                color=[1.0, 1.0, 0.0, 1.0],
                totalMass=0.03,
                volume=20,
                inertiaMatrix=[1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0],
                translation=[0.0, -130.0, 10.0])

    cube.addObject('UncoupledConstraintCorrection')

    return rootNode
