# -*- coding: utf-8 -*-
"""
Step 1:
We are introducing basic mechanical modeling, the new components bring
time integration and a mechanical object to the scene .
"""
import Sofa
from stlib3.scene import Scene
from splib3.animation import animate
from elasticbody import ElasticBody
from fixingbox import FixingBox
from actuatedarm import ActuatedArm


def BluePrint(name="BluePrint"):
    self = Sofa.Core.Node(name)
    self.addObject("MeshSTLLoader", name="loader", filename="../details/data/mesh/blueprint.stl")
    self.addObject("OglModel", name="renderer", src=self.loader.getLinkPath())
    return self


def createScene(rootNode):
    pluginList = ["Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.Mass",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.GUI.Component",
                  "Sofa.Component.Mapping.Linear",
                  "ArticulatedSystemPlugin",
                  "Sofa.Component.Constraint.Projective",
                  "Sofa.Component.Engine.Select",
                  "Sofa.Component.Mapping.NonLinear",
                  "Sofa.Component.SolidMechanics.FEM.Elastic",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.Topology.Container.Constant"]

    # Setting the gravity, assuming the length unit is in millimeters
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], plugins=pluginList, iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    scene.VisualStyle.displayFlags = "showBehavior"
    scene.Settings.mouseButton.stiffness = 10

    # Setting the timestep in seconds
    scene.dt = 0.01

    # Add the blueprint 
    scene.Modelling.addChild(BluePrint())

    # Add the elastic body
    scene.Modelling.addChild(ElasticBody(translation=[0, 30, 0], rotation=[90, 0, 0], color=[1, 1, 1, 0.5]))

    # Fix this elastic body
    box = scene.Modelling.addChild(FixingBox(scene.Modelling,
                                             scene.Modelling.ElasticBody.MechanicalModel,
                                             translation=[0, 30, 0],
                                             scale=[30.0, 30.0, 30.0]))

    box.BoxROI.drawBoxes = True  # < for debugging purpose fix it.

    scene.Modelling.addChild(ActuatedArm(name="arm0", translation=[-100, 0, 0]))

    def myMovement(target, factor):
        target.angleIn = factor

    animate(myMovement, {"target": scene.Modelling.arm0}, duration=1, mode="pingpong")

    # Attach the model we have made into a simulation node 
    # (a node with a time integration scheme and linear solver)        
    scene.Simulation.addChild(scene.Modelling)
