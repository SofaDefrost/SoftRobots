# -*- coding: utf-8 -*-
"""
Step 4:
4-1 Adding the ActuatedArm prefab.
    This prefab is defining the servomotor, the servo-arm and the constraint that attaches the end of the arm to the deformable object.
4-2 Rigidify extremity of deformable part to be able to fix it to the actuated arms
4-3 Fix arms to deformable part
"""
from tutorial import *
# Let's define a Tripod prefab in tripod.py, that we can later call in the createScene function
from tripod import Tripod
from blueprint import Blueprint


def createScene(rootNode):
    from splib3.animation import animate
    from stlib3.scene import Scene
    import math
    
    pluginList = ['ArticulatedSystemPlugin',  # Needed to use components [ArticulatedHierarchyContainer]
                  'Sofa.Component.AnimationLoop',  # Needed to use components [FreeMotionAnimationLoop]
                  'Sofa.Component.Constraint.Lagrangian.Correction',  # Needed to use components [GenericConstraintCorrection]
                  'Sofa.Component.Constraint.Lagrangian.Solver',  # Needed to use components [GenericConstraintSolver]
                  'Sofa.Component.Constraint.Projective',  # Needed to use components [FixedProjectiveConstraint]
                  'Sofa.Component.Engine.Select',  # Needed to use components [BoxROI,SphereROI]
                  'Sofa.Component.LinearSolver.Direct',  # Needed to use components [SparseLDLSolver]
                  'Sofa.Component.Mapping.Linear',  # Needed to use components [BarycentricMapping,SubsetMultiMapping]
                  'Sofa.Component.Mapping.NonLinear',  # Needed to use components [RigidMapping]
                  'Sofa.Component.Mass',  # Needed to use components [UniformMass]
                  'Sofa.Component.SolidMechanics.FEM.Elastic',  # Needed to use components [TetrahedronFEMForceField]
                  'Sofa.Component.SolidMechanics.Spring',  # Needed to use components [RestShapeSpringsForceField]
                  'Sofa.Component.StateContainer',  # Needed to use components [MechanicalObject]
                  'Sofa.Component.Topology.Container.Constant',  # Needed to use components [MeshTopology]
                  'Sofa.Component.Visual',  # Needed to use components [VisualStyle]
                  'Sofa.GUI.Component']  # Needed to use components [AttachBodyButtonSetting]

    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], 
                  plugins=pluginList, iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.Settings.mouseButton.stiffness = 10
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.05
    scene.VisualStyle.displayFlags = "showBehavior"
    scene.dt = 0.01

    # Add the blueprint
    scene.Modelling.addChild(Blueprint())

    # Add the tripod
    tripod = scene.Modelling.addChild(Tripod())
    tripod.BoxROI0.drawBoxes = True
    tripod.BoxROI1.drawBoxes = True
    tripod.BoxROI2.drawBoxes = True

    scene.Simulation.addChild(tripod)

    def myanimate(targets, factor):
        for arm in targets:
            arm.angleIn = -factor * math.pi / 4.

    animate(myanimate, {"targets": [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]}, duration=1)