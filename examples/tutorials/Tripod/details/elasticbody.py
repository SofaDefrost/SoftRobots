# -*- coding: utf-8 -*-
"""
Step 3: Move the content of the ElasticBody in a separated file for reusability
"""
import Sofa


def ElasticBody(name="ElasticBody", rotation=[0, 0, 0], translation=[0, 0, 0], color=[1.0, 1.0, 1.0, 1.0]):
    # To simulate an elastic object, we need:
    # - a deformation law (here linear elasticity)
    # - a solving method (here FEM)
    # - as we are using FEM we need a space discretization (here tetrahedron)
    self = Sofa.Core.Node(name)
    mechanicalmodel = self.addChild("MechanicalModel")
    mechanicalmodel.addObject('GIDMeshLoader',
                              name='loader',
                              rotation=rotation,
                              translation=translation,
                              filename='data/mesh/tripod_low.gidmsh')
    mechanicalmodel.addObject('MeshTopology',
                              src='@loader',
                              name='container')

    mechanicalmodel.addObject('MechanicalObject',
                              name='dofs',
                              position=mechanicalmodel.loader.position.getLinkPath(),
                              showObject=False,
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
    visualmodel = Sofa.Core.Node("VisualModel")
    # Specific loader for the visual model
    visualmodel.addObject('MeshSTLLoader',
                          name='loader',
                          filename='data/mesh/tripod_mid.stl',
                          rotation=rotation,
                          translation=translation)
    visualmodel.addObject('OglModel',
                          src=visualmodel.loader.getLinkPath(),
                          name='renderer',
                          color=color)
    self.addChild(visualmodel)

    visualmodel.addObject('BarycentricMapping',
                          input=mechanicalmodel.dofs.getLinkPath(),
                          output=visualmodel.renderer.getLinkPath())
    return self


def createScene(rootNode):
    from stlib3.scene import Scene

    pluginsList = [
        'Sofa.Component.IO.Mesh',  # Needed to use components [GIDMeshLoader,MeshSTLLoader]
        'Sofa.Component.LinearSolver.Direct',  # Needed to use components [SparseLDLSolver]
        'Sofa.Component.Mapping.Linear',  # Needed to use components [BarycentricMapping]
        'Sofa.Component.Mass',  # Needed to use components [UniformMass]
        'Sofa.Component.SolidMechanics.FEM.Elastic',  # Needed to use components [TetrahedronFEMForceField]
        'Sofa.Component.StateContainer',  # Needed to use components [MechanicalObject]
        'Sofa.Component.Topology.Container.Constant',  # Needed to use components [MeshTopology]
        'Sofa.Component.Visual',  # Needed to use components [VisualStyle]
        'Sofa.GL.Component.Rendering3D',  # Needed to use components [OglModel,OglSceneFrame]
        'Sofa.GUI.Component',  # Needed to use components [AttachBodyButtonSetting]
    ]

    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0], plugins=pluginsList,
                  iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    scene.Simulation.addChild(ElasticBody())
