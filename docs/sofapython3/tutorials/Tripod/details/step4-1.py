# -*- coding: utf-8 -*-
"""
Step 4-1: Adding the ActuatedArm prefab.
This prefab is defining the servomotor, the servo-arm and the constraint that attaches the end of the arm to the deformable object.
"""
import Sofa
from tutorial import *
from splib3.numerics import sin, cos, to_radians
from stlib3.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm


def ElasticBody(parent):
    body = parent.addChild("ElasticBody")

    elasticMaterialObject = body.addChild(ElasticMaterialObject(volumeMeshFileName="data/mesh/tripod_low.gidmsh",
                                                                translation=[0.0, 30, 0.0], rotation=[90, 0, 0],
                                                                youngModulus=250, poissonRatio=0.45, totalMass=0.032))

    visual = body.addChild("Visual")
    visual.addObject("MeshSTLLoader", name="loader", filename="data/mesh/tripod_mid.stl")
    visual.addObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                     rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.addObject("BarycentricMapping",
                     input=body.ElasticMaterialObject.dofs.getLinkPath(),
                     output=visual.renderer.getLinkPath())

    return elasticMaterialObject


class Tripod(Sofa.Prefab):
    properties = [
        {'name': 'name', 'type': 'string', 'help': 'Node name', 'default': 'Tripod'},
        {'name': 'radius', 'type': 'int', 'help': 'Rotation', 'default': 60},
        {'name': 'numMotors', 'type': 'int', 'help': 'Translation', 'default': 3},
        {'name': 'angleShift', 'type': 'float', 'help': 'Translation', 'default': 180.0}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.elasticMaterialObject = ElasticBody(self)

        dist = self.radius.value
        numstep = self.numMotors.value
        self.actuatedarms = []
        for i in range(0, numstep):
            name = "ActuatedArm" + str(i)
            translation, eulerRotation = self.__getTransform(i, numstep, self.angleShift.value, self.radius.value, dist)
            arm = self.addChild(ActuatedArm(name=name, translation=translation, rotation=eulerRotation))
            self.actuatedarms.append(arm)
            # Add limits to angle that correspond to limits on real robot
            arm.ServoMotor.minAngle = -2.0225
            arm.ServoMotor.maxAngle = -0.0255

    def __getTransform(self, index, numstep, angleShift, radius, dist):
        fi = float(index)
        fnumstep = float(numstep)
        angle = fi * 360 / fnumstep
        angle2 = fi * 360 / fnumstep + angleShift
        eulerRotation = [0, angle, 0]
        translation = [dist * sin(to_radians(angle2)), -1.35, dist * cos(to_radians(angle2))]

        return translation, eulerRotation


def createScene(rootNode):
    from splib3.animation import animate
    from stlib3.scene import Scene
    from fixingbox import FixingBox
    import math

    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], iterative=False,
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaDeformable', 'SofaEngine',
                           'SofaGeneralRigid', 'SofaRigid', 'SofaGraphComponent', 'SofaBoundaryCondition'])
    scene.addMainHeader()
    scene.addObject('AttachBodyButtonSetting', stiffness=10)  # Set mouse spring stiffness
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    rootNode.dt = 0.01

    scene.VisualStyle.displayFlags = "showBehavior"

    tripod = scene.Modelling.addChild(Tripod())
    FixingBox(tripod, tripod.ElasticBody.ElasticMaterialObject, scale=[10, 10, 10], translation=[0., 25, 0.])
    tripod.FixingBox.BoxROI.drawBoxes = True

    motors = scene.Simulation.addChild("Motors")
    motors.addChild(tripod.ActuatedArm0)
    motors.addChild(tripod.ActuatedArm1)
    motors.addChild(tripod.ActuatedArm2)

    def myanimate(targets, factor):
        for arm in targets:
            arm.angleIn.value = -factor * math.pi / 4

    animate(myanimate, {"targets": [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]}, duration=1)