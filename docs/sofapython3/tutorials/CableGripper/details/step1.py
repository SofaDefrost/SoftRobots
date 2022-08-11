# -*- coding: utf-8 -*-
from stlib3.physics.rigid import Floor
from stlib3.physics.rigid import Cube


def createScene(rootNode):
    """This is my first scene"""
    rootNode.addObject('VisualStyle', displayFlags="showBehavior showCollisionModels")

    Floor(rootNode,
          translation=[0.0, -160.0, 0.0],
          isAStaticObject=True)

    Cube(rootNode,
         translation=[0.0, 0.0, 0.0],
         uniformScale=20,
         color=[1.0, 0.0, 0.0, 1.0])

    return rootNode
