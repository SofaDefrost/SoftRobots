# -*- coding: utf-8 -*-
'''
Step 3: Move the content of the ElasticBody in a separated file for reusability
'''
import Sofa

def ElasticBody():
    # To simulate an elastic object, we need:
    # - a deformation law (here linear elasticity)
    # - a solving method (here FEM)
    # - as we are using FEM we need a space discretization (here tetrahedron)
    self = Sofa.Core.Node('ElasticBody')

    # Specific loader for the mechanical model
    self.addObject('GIDMeshLoader',
                          name='loader',
                          filename='data/mesh/tripod_low.gidmsh')
    self.addObject('TetrahedronSetTopologyContainer',
                          src='@loader',
                          name='tetras')
                          
    self = elasticbody.addChild("MechanicalModel")                      
    self.addObject('MechanicalObject',
                          name='dofs',
                          position=elasticbody.loader.position.getLinkPath(),
                          rotation=[90.0, 0.0, 0.0],
                          showObject=True,
                          showObjectScale=5.0)
    self.addObject('UniformMass', 
                          name="mass", 
                          totalMass=0.032)

    # ForceField components
    self.addObject('TetrahedronFEMForceField',
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
    return self
   
                     
