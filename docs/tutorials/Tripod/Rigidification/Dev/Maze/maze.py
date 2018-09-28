# -*- coding: utf-8 -*-
""" Model for a maze coaster
"""
import Sofa
from splib.numerics import RigidDof
from splib.objectmodel import *

from stlib.visuals import VisualModel
from stlib.solver import DefaultSolver
from stlib.scene import Scene, Node, get

@SofaPrefab
class Maze(SofaObject):
    """A circular maze coaster

    This prefab is implementing a maze coaster.
    """
    def __init__(self, parent,
                 translation=[0.0,0.0,0.0], rotation=[0.0,0.0,0.0], scale=[1.0,1.0,1.0], doAddVisualModel=True):
        self.node = Node(parent, "Maze")
       
        self.dofs = self.node.createObject("MechanicalObject", size=1,
                                          name="dofs",
                                          translation=translation, rotation=rotation, scale=scale,
                                          template='Rigid', showObject=True, showObjectScale=15)
        totalMass = 1.0
        volume = 1.0
        inertiaMatrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        #self.vertexMass = self.node.createObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])


        if doAddVisualModel:
            self.addVisualModel()

    def addVisualModel(self):
        self.visualmodel = VisualModel(self.node, 'data/Maze_Coaster.stl')
        self.visualmodel.node.createObject('RigidMapping', name = "mapping")

#class MazeUpdater(Sofa.PythonScriptController):
    ## voir scene controller


def createScene(rootNode):
    from splib.animation import animate
    from splib.animation.easing import LinearRamp
    from splib.scenegraph import get

    from stlib.scene import MainHeader
    Scene(rootNode)
    s=DefaultSolver(rootNode)

    maze = Maze(rootNode, translation=[2,0,0])