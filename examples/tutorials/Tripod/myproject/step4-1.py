# -*- coding: utf-8 -*-
"""
Step 4-1: Adding the ActuatedArm prefab.
This prefab is defining the servomotor, the servo-arm and the constraint that attaches the end of the arm to the deformable object.
"""
import Sofa
from tutorial import *
from splib3.numerics import sin, cos, to_radians
from actuatedarm import ActuatedArm
from elasticbody import ElasticBody
from blueprint import Blueprint


def Tripod(name="Tripod", radius=60, numMotors=3, angleShift=180.0):
    def __getTransform(index, numstep, angleShift, radius, dist):
        fi = float(index)
        fnumstep = float(numstep)
        angle = fi * 360 / fnumstep
        angle2 = fi * 360 / fnumstep + angleShift
        eulerRotation = [0, angle, 0]
        translation = [dist * sin(to_radians(angle2)), -1.35, dist * cos(to_radians(angle2))]

        return translation, eulerRotation

    self = Sofa.Core.Node(name)
    self.actuatedarms = []
    for i in range(0, numMotors):
        name = "ActuatedArm" + str(i)
        translation, eulerRotation = __getTransform(i, numMotors, angleShift, radius, radius)
        arm = ActuatedArm(name=name, translation=translation, rotation=eulerRotation)

        # Add limits to angle that correspond to limits on real robot
        arm.ServoMotor.minAngle = -2.0225
        arm.ServoMotor.maxAngle = -0.0255
        self.actuatedarms.append(arm)
        self.addChild(arm)

    self.addChild(ElasticBody(translation=[0.0, 30, 0.0], rotation=[90, 0, 0], color=[1.0, 1.0, 1.0, 0.5]))
    return self


def createScene(rootNode):
    from splib3.animation import animate
    from stlib3.scene import Scene
    from fixingbox import FixingBox
    import math

    pluginList = ["ArticulatedSystemPlugin",
                  "Sofa.Component.AnimationLoop",
                  "Sofa.Component.Constraint.Lagrangian.Correction",
                  "Sofa.Component.Constraint.Lagrangian.Solver",
                  "Sofa.Component.Constraint.Projective",
                  "Sofa.Component.Engine.Select",
                  "Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.Mass",
                  "Sofa.Component.SolidMechanics.FEM.Elastic",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Constant",
                  "Sofa.Component.Topology.Container.Dynamic",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.GUI.Component",
                  "Sofa.Component.Mapping.Linear",
                  "Sofa.Component.Mapping.NonLinear"]

    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], iterative=False,
                  plugins=pluginList)
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.Settings.mouseButton.stiffness = 10
    rootNode.dt = 0.01
    scene.VisualStyle.displayFlags = "showBehavior"

    scene.Modelling.addChild(Blueprint())

    tripod = scene.Modelling.addChild(Tripod())

    FixingBox(scene.Modelling, tripod.ElasticBody.MechanicalModel, scale=[10, 10, 10], translation=[0., 25, 0.])
    scene.Modelling.FixingBox.BoxROI.drawBoxes = True

    scene.Simulation.addChild(scene.Modelling.Tripod)
    scene.Simulation.addChild(scene.Modelling.FixingBox)

    # Add animations
    def myanimate(targets, factor):
        for arm in targets:
            arm.angleIn.value = -factor * math.pi / 4

    animate(myanimate, {"targets": [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]}, duration=1)
