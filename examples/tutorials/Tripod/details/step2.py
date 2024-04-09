# -*- coding: utf-8 -*-
"""
Step 2: Introducing elastic material modelling
"""
import Sofa
from stlib3.scene import Scene


def createScene(rootNode):
    pluginList = ["Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.Mass",
                  "Sofa.Component.SolidMechanics.FEM.Elastic",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Dynamic",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.GUI.Component",
                  "Sofa.Component.Mapping.Linear"]

    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0],
                  plugins=pluginList, iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    scene.dt = 0.01

    # It is possible to visualize the 'forcefields' by doing
    scene.VisualStyle.displayFlags = 'showForceFields'

    #  Change the stiffness of the spring while interacting with the simulation
    scene.Settings.mouseButton.stiffness = 1.0

    # Graphic modelling of the legends associated to the servomotors
    blueprint = Sofa.Core.Node("Blueprints")
    blueprint.addObject('MeshSTLLoader', name='loader', filename='data/mesh/blueprint.stl')
    blueprint.addObject('OglModel', src='@loader')
    scene.Modelling.addChild(blueprint)

    # To simulate an elastic object, we need:
    # - a deformation law (here linear elasticity)
    # - a solving method (here FEM)
    # - as we are using FEM we need a space discretization (here tetrahedron)
    elasticbody = scene.Modelling.addChild('ElasticBody')

    # Specific loader for the mechanical model
    elasticbody.addObject('GIDMeshLoader',
                          name='loader',
                          filename='data/mesh/tripod_low.gidmsh')
    elasticbody.addObject('TetrahedronSetTopologyContainer',
                          src='@loader',
                          name='tetras')

    mechanicalmodel = elasticbody.addChild("MechanicalModel")
    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              position=elasticbody.loader.position.getLinkPath(),
                              rotation=[90.0, 0.0, 0.0],
                              showObject=True,
                              showObjectScale=5.0)
    mechanicalmodel.addObject('UniformMass',
                              name="mass",
                              totalMass=0.032)

    # ForceField components
    mechanicalmodel.addObject('TetrahedronFEMForceField',
                              name="linearElasticBehavior",
                              youngModulus=250,
                              poissonRatio=0.45)

    # Visual model
    visual = Sofa.Core.Node("VisualModel")
    # Specific loader for the visual model
    visual.addObject('MeshSTLLoader',
                     name='loader',
                     filename='data/mesh/tripod_mid.stl',
                     rotation=[90, 0, 0])
    visual.addObject('OglModel',
                     src=visual.loader.getLinkPath(),
                     name='renderer',
                     color=[1.0, 1.0, 1.0, 0.5])
    scene.Modelling.ElasticBody.addChild(visual)

    visual.addObject('BarycentricMapping',
                     input=mechanicalmodel.dofs.getLinkPath(),
                     output=visual.renderer.getLinkPath())

    scene.Simulation.addChild(elasticbody)
