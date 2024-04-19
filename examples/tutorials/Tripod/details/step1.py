# -*- coding: utf-8 -*-
"""
Step 1:
We are introducing basic mechanical modeling, the new components bring
time integration and a mechanical object to the scene .
"""
import Sofa
from stlib3.scene import Scene


def createScene(rootNode):
    pluginList = ["Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.Mass",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.GUI.Component",
                  "Sofa.Component.Mapping.Linear"]

    # Setting the gravity, assuming the length unit is in millimeters
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], plugins=pluginList, iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')

    # Setting the timestep in seconds
    rootNode.dt = 0.01

    # Graphic modelling of the legends associated to the servomotors
    blueprint = Sofa.Core.Node("Blueprints")
    blueprint.addObject('MeshSTLLoader', name='loader', filename='data/mesh/blueprint.stl')
    blueprint.addObject('OglModel', src='@loader')
    scene.Settings.addChild(blueprint)

    # Tool to load the mesh file of the silicone piece. It will be used for both the mechanical and the visual models.
    scene.Modelling.addObject('MeshSTLLoader', name='loader', filename='data/mesh/tripod_mid.stl')

    # Basic mechanical modelling of the silicone piece
    elasticbody = scene.Modelling.addChild('MechanicalModel')
    elasticbody.addObject('MechanicalObject', name='dofs',
                          position=scene.Modelling.loader.position.getLinkPath(),
                          showObject=True, showObjectScale=5.0,
                          rotation=[90.0, 0.0, 0.0])
    elasticbody.addObject('UniformMass', totalMass='1.0')

    # Visual object
    visual = scene.Modelling.addChild('VisualModel')
    # The mesh used for the Visual object is the same as the one for the MechanicalObject, and has been introduced in
    # the rootNode
    visual.addObject('OglModel', name='renderer',
                     src=scene.Modelling.loader.getLinkPath(),
                     color=[1.0, 1.0, 1.0, 0.5])

    # A mapping applies the deformations computed on the mechanical model (the input parameter)
    # to the visual model (the output parameter).
    visual.addObject('IdentityMapping',
                     input=elasticbody.dofs.getLinkPath(),
                     output=visual.renderer.getLinkPath())

    scene.Simulation.addChild(scene.Modelling)
