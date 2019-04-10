# -*- coding: utf-8 -*-
"""
Step 4-1: Adding the ActuatedArm prefab.
This prefab is defining the servomotor, the servo-arm and the constraint that attaches the end of the arm to the deformable object.
"""
from tutorial import *
from splib.numerics import sin, cos, to_radians
from stlib.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
from splib.objectmodel import SofaPrefab, SofaObject


def ElasticBody(parent):
    body = parent.createChild("ElasticBody")

    e = ElasticMaterialObject(body,
                              volumeMeshFileName="data/mesh/tripod_mid.gidmsh",
                              translation=[0.0, 30, 0.0], rotation=[90, 0, 0],
                              youngModulus=800, poissonRatio=0.45, totalMass=0.032)

    visual = body.createChild("Visual")
    visual.createObject("MeshSTLLoader", name="loader", filename="data/mesh/tripod_mid.stl")
    visual.createObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                        rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.createObject("BarycentricMapping",
                        input=e.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    return body


@SofaPrefab
class Tripod(SofaObject):

    def __init__(self, parent, name="Tripod", radius=60, numMotors=3, angleShift=180.0):
        self.node = parent.createChild(name)
        ElasticBody(self.node)

        dist = radius
        numstep = numMotors
        self.actuatedarms = []
        for i in range(0, numstep):
            name = "ActuatedArm"+str(i)
            translation, eulerRotation = self.__getTransform(i, numstep, angleShift, radius, dist)
            arm = ActuatedArm(self.node, name=name,
                              translation=translation, eulerRotation=eulerRotation)
            self.actuatedarms.append(arm)
            # Add limits to angle that correspond to limits on real robot
            arm.ServoMotor.minAngle = -2.0225
            arm.ServoMotor.maxAngle = -0.0255

    def __getTransform(self, index, numstep, angleShift, radius, dist):
        fi = float(index)
        fnumstep = float(numstep)
        angle = fi*360/fnumstep
        angle2 = fi*360/fnumstep+angleShift
        eulerRotation = [0, angle, 0]
        translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]

        return translation, eulerRotation


def createScene(rootNode):
    from splib.animation import animate
    from fixingbox import FixingBox
    import math

    scene = Scene(rootNode)

    scene.VisualStyle.displayFlags = "showBehavior"

    tripod = Tripod(scene.Modelling)
    FixingBox(tripod, tripod.ElasticBody.ElasticMaterialObject, scale=[10, 10, 10], translation=[0., 25, 0.])
    tripod.FixingBox.BoxROI.drawBoxes = True

    scene.Simulation.addChild(tripod.ActuatedArm0)
    scene.Simulation.addChild(tripod.ActuatedArm1)
    scene.Simulation.addChild(tripod.ActuatedArm2)

    def myanimate(targets, factor):
        for arm in targets:
            arm.ServoMotor.angleIn = -factor*math.pi/4.

    animate(myanimate, {"targets": [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]}, duration=1)
