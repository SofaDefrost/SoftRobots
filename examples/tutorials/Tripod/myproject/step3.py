# -*- coding: utf-8 -*-
"""
Step 3:
In this step, the use of code blocks using functions is introduced, as reusable elements of the code.
"""
from stlib3.scene import Scene

# A prefab to fix some part of an object at its rest position.
from fixingbox import FixingBox

#  A prefab to fix
from elasticbody import ElasticBody

from blueprint import Blueprint


def createScene(rootNode):
    pluginList = ["Sofa.Component.Engine.Select",
                  "Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.Mass",
                  "Sofa.Component.SolidMechanics.FEM.Elastic",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Dynamic",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.GUI.Component",
                  "Sofa.Component.Mapping.Linear",
                  "Sofa.Component.Topology.Container.Constant"]

    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0],
                  plugins=pluginList,
                  iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    scene.Settings.mouseButton.stiffness = 10
    scene.VisualStyle.displayFlags = 'showBehavior'
    scene.dt = 0.01

    # Add the blueprint prefab
    scene.Modelling.addChild(Blueprint())

    # Add the elasticobject prefab
    scene.Modelling.addChild(ElasticBody(rotation=[90, 0, 0], color=[1.0, 1.0, 1.0, 0.5]))

    # Instanciating the FixingBox prefab into the graph, constraining the mechanical object of the ElasticBody.
    box = FixingBox(scene.Modelling,
                    scene.Modelling.ElasticBody.MechanicalModel,
                    translation=[10.0, 0.0, 0.0],
                    scale=[30., 30., 30.])
    # Changing the property of the Box ROI so that the constraint area appears on screen.
    box.BoxROI.drawBoxes = True

    scene.Simulation.addChild(scene.Modelling.ElasticBody)
    scene.Simulation.addChild(scene.Modelling.FixingBox)
