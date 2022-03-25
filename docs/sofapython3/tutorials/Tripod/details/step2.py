# -*- coding: utf-8 -*-
'''
Step 2: Introducing elastic material modelling
'''
from stlib3.scene import Scene


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0],
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaGraphComponent'], iterative=False)
    scene.addMainHeader()
    scene.addObject('AttachBodyButtonSetting', stiffness=10)  # Set mouse spring stiffness
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    rootNode.dt = 0.01

    # Graphic modelling of the legends associated to the servomotors
    scene.addChild('Config')
    scene.Config.addObject('MeshSTLLoader', name='loader', filename='data/mesh/blueprint.stl')
    scene.Config.addObject('OglModel', src='@loader')

    # To simulate an elastic object, we need:
    # - a deformation law (here linear elasticity)
    # - a solving method (here FEM)
    # - as we are using FEM we need a space discretization (here tetrahedron)
    elasticbody = rootNode.Simulation.addChild('ElasticBody')

    # Specific loader for the mechanical model
    elasticbody.addObject('GIDMeshLoader',
                          name='loader',
                          filename='data/mesh/tripod_low.gidmsh')
    elasticbody.addObject('TetrahedronSetTopologyContainer',
                          src='@loader',
                          name='container')
    elasticbody.addObject('MechanicalObject',
                          name='dofs',
                          position=elasticbody.loader.position.getLinkPath(),
                          rotation=[90.0, 0.0, 0.0],
                          showObject=True,
                          showObjectScale=5.0)
    elasticbody.addObject('UniformMass', totalMass=0.032)

    # ForceField components
    elasticbody.addObject('TetrahedronFEMForceField',
                          youngModulus=250,
                          poissonRatio=0.45)
    # It is possible to visualize the 'forcefields' by doing
    scene.VisualStyle.displayFlags = 'showForceFields'

    # Visual model
    visual = elasticbody.addChild('Visual')

    # Specific loader for the visual model
    visual.addObject('MeshSTLLoader',
                     name='loader',
                     filename='data/mesh/tripod_mid.stl',
                     rotation=[90, 0, 0])
    visual.addObject('OglModel',
                     src='@loader',
                     name='renderer',
                     color=[1.0, 1.0, 1.0, 0.5])

    visual.addObject('BarycentricMapping',
                     input=elasticbody.dofs.getLinkPath(),
                     output=visual.renderer.getLinkPath())