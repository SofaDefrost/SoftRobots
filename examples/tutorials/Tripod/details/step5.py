# -*- coding: utf-8 -*-
"""
Step 5: Adding a controller.
The controller will connect user actions to the simulated behaviour.
"""
import Sofa
from stlib3.scene import Scene                   #< Prefab for the scene
from tripod import Tripod                        #< Prefab for the Tripod
from tripodcontroller import TripodController    #< Implementation of a controller that modify the Tripod


class MyController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        def onKeypressedEvent(self, key):
            print("Key Pressed")


def createScene(rootNode):

    pluginList = ["ArticulatedSystemPlugin",
                  "Sofa.Component.AnimationLoop",
                  "Sofa.Component.Constraint.Lagrangian.Correction",
                  "Sofa.Component.Constraint.Lagrangian.Solver",
                  "Sofa.Component.Constraint.Projective",
                  "Sofa.Component.Engine.Select",
                  "Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.Mapping.MappedMatrix",
                  "Sofa.Component.Mass",
                  "Sofa.Component.SolidMechanics.FEM.Elastic",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Constant",
                  "Sofa.Component.Topology.Container.Dynamic",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.GUI.Component", "Sofa.Component.Mapping.Linear", "Sofa.Component.Mapping.NonLinear"]

    scene = Scene(rootNode, gravity=[0., -9810., 0.], dt=0.01, iterative=False, plugins=pluginList)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.005
    scene.Settings.mouseButton.stiffness = 10
    scene.VisualStyle.displayFlags = "showBehavior"

    tripod = Tripod()
    scene.Modelling.addChild(tripod)
    scene.Modelling.addObject(TripodController(name="TripodController",actuators=[tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]))
    scene.Simulation.addChild(tripod)
