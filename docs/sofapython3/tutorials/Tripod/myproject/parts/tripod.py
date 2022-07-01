# -*- coding: utf-8 -*-
"""
Tripod class
"""
from splib3.numerics import to_radians
from stlib3.physics.collision import CollisionMesh
from stlib3.physics.mixedmaterial import Rigidify
from stlib3.components import addOrientedBoxRoi
from splib3.numerics import vec3
from splib3.numerics.quat import Quat
from tutorial import *
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

    def __rigidify(self, radius=60, numMotors=3, angleShift=180.0):
        deformableObject = self.ElasticBody.MechanicalModel
        self.ElasticBody.init()
        dist = radius
        numstep = numMotors
        groupIndices = []
        frames = []
        for i in range(0, numstep):
            translation, eulerRotation = __getTransform(i, numstep, angleShift, radius, dist)

            box = addOrientedBoxRoi(self, position=[list(i) for i in deformableObject.dofs.rest_position.value],
                                    name="BoxROI" + str(i),
                                    translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                                    eulerRotation=eulerRotation, scale=[45, 15, 30])

            box.drawBoxes = False
            box.init()
            groupIndices.append([ind for ind in box.indices.value])
            frames.append(vec3.vadd(translation, [0.0, 25.0, 0.0]) + list(
                Quat.createFromEuler([0, float(i) * 360 / float(numstep), 0], inDegree=True)))

        effectorPos = [0, 30, 0]
        o = deformableObject.addObject('SphereROI', name='roi', template='Rigid3',
                                                    centers=effectorPos, radii=[7.5], drawSphere=True)
        o.init()
        groupIndices.append(list(o.indices.value))

        frames.append([effectorPos[0], effectorPos[1],
                       effectorPos[2], 0, 0, 0, 1])

        # Rigidify the deformable part at extremity to attach arms
        rigidifiedstruct = Rigidify(self, deformableObject, groupIndices=groupIndices, frames=frames,
                                    name="RigidifiedStructure")

    def __attachToActuatedArms(self, numstep):

         rigidParts = self.RigidifiedStructure.RigidParts
         for arm in self.actuatedarms:
            arm.ServoMotor.Articulation.ServoWheel.addChild(rigidParts)

         freeCenter = self.RigidifiedStructure.addChild('FreeCenter')
         freeCenter.addObject('MechanicalObject', name="dofs", template="Rigid3", position=[0, 30, 0, 0, 0, 0, 1], showObject=False, showObjectScale=10)
         freeCenter.addChild(rigidParts)

         rigidParts.addObject('SubsetMultiMapping',
                              name="mapping",
                              input=[self.actuatedarms[0].ServoMotor.Articulation.ServoWheel.getLinkPath(),
                                     self.actuatedarms[1].ServoMotor.Articulation.ServoWheel.getLinkPath(),
                                     self.actuatedarms[2].ServoMotor.Articulation.ServoWheel.getLinkPath(),
                                     freeCenter.getLinkPath()],
                              output='@./', indexPairs=[0, 1, 1, 1, 2, 1,3,0])

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

    self.addChild(ElasticBody(translation=[0.0, 30, 0.0], rotation=[90,0,0], color=[1.0,1.0,1.0,0.5]))
    __rigidify(self, radius, numMotors, angleShift)
    __attachToActuatedArms(self, numMotors)

    def addCollision():
            CollisionMesh(self.ElasticBody.MechanicalModel,
                        surfaceMeshFileName="data/mesh/tripod_low.stl", name="CollisionModel",
                        translation=[0.0, 30, 0.0], rotation=[90, 0, 0], collisionGroup=1)

    self.addCollision = addCollision

    return self


def createScene(rootNode):
    from splib3.animation import animate
    from stlib3.scene import Scene
    import math

    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], iterative=False,
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaDeformable', 'SofaEngine',
                           'SofaGeneralRigid', 'SofaMiscMapping', 'SofaRigid', 'SofaGraphComponent', 'SofaBoundaryCondition',
                           'SofaGeneralAnimationLoop', 'SofaConstraint'])
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

    # Use this to activate some rendering on the rigidified object ######################################
    setData(tripod.RigidifiedStructure.RigidParts.dofs, showObject=True, showObjectScale=10, drawMode=2)
    setData(tripod.RigidifiedStructure.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=1,
            drawMode=1, showColor=[1., 1., 0., 1.])
    setData(tripod.RigidifiedStructure.DeformableParts.dofs, showObject=True, showObjectScale=1, drawMode=2)
    #####################################################################################################

    scene.Simulation.addChild(tripod)

    def myanimate(targets, factor):
        for arm in targets:
            arm.angleIn = -factor * math.pi / 4.

    animate(myanimate, {"targets": [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]}, duration=1)

    # Temporary additions to have the system correctly built in SOFA
    # Will no longer be required in SOFA v22.06
    scene.Simulation.addObject('MechanicalMatrixMapper',
                                 name="deformableAndFreeCenterCoupling",
                                 template='Vec3,Rigid3',
                                 object1=tripod["RigidifiedStructure.DeformableParts.dofs"].getLinkPath(),
                                 object2=tripod["RigidifiedStructure.FreeCenter.dofs"].getLinkPath(),
                                 nodeToParse=tripod["RigidifiedStructure.DeformableParts.MechanicalModel"].getLinkPath())

    # Temporary additions to have the system correctly built in SOFA
    # Will no longer be required in SOFA v22.06
    for i in range(3):
        scene.Simulation.addObject('MechanicalMatrixMapper',
                                   name="deformableAndArm{i}Coupling".format(i=i),
                                   template='Vec1,Vec3',
                                   object1=tripod["ActuatedArm" + str(i) + ".ServoMotor.Articulation.dofs"].getLinkPath(),
                                   object2=tripod["RigidifiedStructure.DeformableParts.dofs"].getLinkPath(),
                                   skipJ2tKJ2=False if i == 0 else True,
                                   nodeToParse=tripod["RigidifiedStructure.DeformableParts.MechanicalModel"].getLinkPath())
