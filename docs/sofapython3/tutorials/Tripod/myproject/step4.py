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

    scene.Simulation.addChild(tripod)

    def myanimate(targets, factor):
        for arm in targets:
            arm.angleIn = -factor * math.pi / 4.

    animate(myanimate, {"targets": [tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]}, duration=1)

    # Temporary additions to have the system correctly built in SOFA
    # Will no longer be required in SOFA v23.12
    scene.Simulation.addObject('MechanicalMatrixMapper',
                                 name="deformableAndFreeCenterCoupling",
                                 template='Vec3,Rigid3',
                                 object1=tripod["RigidifiedStructure.DeformableParts.dofs"].getLinkPath(),
                                 object2=tripod["RigidifiedStructure.FreeCenter.dofs"].getLinkPath(),
                                 nodeToParse=tripod["RigidifiedStructure.DeformableParts.MechanicalModel"].getLinkPath())

    for i in range(3):
        scene.Simulation.addObject('MechanicalMatrixMapper',
                                   name="ArmAndDeformableCoupling" + str(i),
                                   template='Vec1,Vec3',
                                   object1=tripod["ActuatedArm" + str(i) + ".ServoMotor.Articulation.dofs"].getLinkPath(),
                                   object2=tripod["RigidifiedStructure.DeformableParts.dofs"].getLinkPath(),
                                   skipJ2tKJ2=False if i == 0 else True,
                                   nodeToParse=tripod.RigidifiedStructure.DeformableParts.MechanicalModel.getLinkPath())
