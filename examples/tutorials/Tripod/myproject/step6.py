# -*- coding: utf-8 -*-
"""
Step 6: we are showing how to add collision detection
and contact management in our project.

The important component to see in the graph are:
- the ones added by the addContact function
- the one in the "Collision" part of the tripod object
- the one in the "Collision" part of the Sphere object
"""
import Sofa
from splib3.constants import Key
from stlib3.physics.rigid import Sphere
from stlib3.scene.contactheader import ContactHeader
from stlib3.scene import Scene
from tripod import Tripod
from tripodcontroller import TripodController


class JumpController(Sofa.Core.Controller):
    """This controller has two roles:
       - if the user presses up/left/right/down/plus/minus, the servomotor angle
         is changed.
       - if the user presses A, an animation is started to move the servomotor to the initial position
         of the real robot.
    """

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.stepsize = 0.1
        self.actuators = kwargs["actuators"]

    def onKeypressedEvent(self, event):
        key = event['key']
        self.animateTripod(key)

    def animateTripod(self, key):
        apos = None
        if key == Key.Z:
            apos = -3.14 / 4
        if key == Key.Q:
            apos = -3.14 / 3

        if apos is not None:
            for actuator in self.actuators:
                actuator.ServoMotor.angleIn = apos


def createScene(rootNode):
    pluginList = ["ArticulatedSystemPlugin",
                  "Sofa.Component.AnimationLoop",
                  "Sofa.Component.Collision.Detection.Algorithm",
                  "Sofa.Component.Collision.Detection.Intersection",
                  "Sofa.Component.Collision.Geometry",
                  "Sofa.Component.Collision.Response.Contact",
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

    scene = Scene(rootNode, gravity=[0., -9810., 0.], dt=0.01, plugins=pluginList, iterative=False)

    ContactHeader(rootNode, alarmDistance=15, contactDistance=0.5, frictionCoef=0.2)

    # Adding contact handling
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.VisualStyle.displayFlags = "showCollisionModels"
    scene.Simulation.TimeIntegrationSchema.rayleighStiffness = 0.005
    scene.Settings.mouseButton.stiffness = 10

    tripod = scene.Modelling.addChild(Tripod())
    tripod.addCollision()

    # The regular controller that is being used for the last 2 steps
    controller = scene.addObject(TripodController(name="TripodController",
                                                  actuators=[tripod.ActuatedArm0, tripod.ActuatedArm1,
                                                             tripod.ActuatedArm2]))
    # You can set the animation from the python script by adding this call
    controller.initTripod('A')

    # The additionnal controller that add two predefined positions for the three servomotors
    scene.addObject(JumpController(name="JumpController",
                                   actuators=[tripod.ActuatedArm0, tripod.ActuatedArm1, tripod.ActuatedArm2]))

    sphere = Sphere(scene.Modelling, translation=[0.0, 50.0, 0.0],
                    uniformScale=13.,
                    totalMass=0.032,
                    isAStaticObject=True)
    sphere.addObject('UncoupledConstraintCorrection')

    scene.Simulation.addChild(sphere)
    scene.Simulation.addChild(tripod)
