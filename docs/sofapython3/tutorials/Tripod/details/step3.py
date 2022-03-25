# -*- coding: utf-8 -*-
'''
Step 3:
In this step, the use of code blocks using functions is introduced, as reusable elements of the code.
'''
from stlib3.scene import Scene
# A prefab that implements an ElasticMaterialObject
from stlib3.physics.deformable import ElasticMaterialObject
# A prefab to fix some part of an object at its rest position.
from fixingbox import FixingBox


# This function includes the whole mechanical model of the silicone piece, as written in the previous step, except that the prefab ElasticMaterialObject is used, instead of creating each component.
def ElasticBody(parent):
    body = parent.addChild('ElasticBody')

    # Prefab ElasticMaterialObject implementing the whole mechanical model of the silicone piece
    e = body.addChild(ElasticMaterialObject(
        volumeMeshFileName='data/mesh/tripod_low.gidmsh',
        poissonRatio=0.45,
        youngModulus=250,
        totalMass=0.032,
        rotation=[90, 0, 0]))

    # Visual model
    visual = e.addChild('Visual')
    visual.addObject('MeshSTLLoader',
                     name='loader',
                     filename='data/mesh/tripod_mid.stl',
                     rotation=[90, 0, 0])
    visual.addObject('OglModel',
                     name='renderer',
                     src='@loader',
                     color=[1.0, 1.0, 1.0, 0.5])
    visual.addObject('BarycentricMapping',
                     input=e.dofs.getLinkPath(),
                     output=visual.renderer.getLinkPath())
    return body


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0],
                  plugins=['SofaSparseSolver', 'SofaOpenglVisual', 'SofaSimpleFem', 'SofaDeformable', 'SofaEngine', 'SofaGraphComponent'],
                  iterative=False)
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('AttachBodyButtonSetting', stiffness=10)  # Set mouse spring stiffness
    rootNode.dt = 0.01
    scene.VisualStyle.displayFlags = 'showBehavior'

    scene.addChild('Config')
    scene.Config.addObject('MeshSTLLoader', name='loader', filename='data/mesh/blueprint.stl')
    scene.Config.addObject('OglModel', src='@loader')

    # Instanciating the prefab into the graph
    body = ElasticBody(rootNode.Simulation)

    # Instanciating the FixingBox prefab into the graph, constraining the mechanical object of the ElasticBody.
    fix = FixingBox(rootNode.Simulation,
                    body.ElasticMaterialObject,
                    translation=[10.0, 0.0, 0.0],
                    scale=[30., 30., 30.])

    # Changing the property of the Box ROI so that the constraint area appears on screen.
    rootNode.Simulation.FixingBox.BoxROI.drawBoxes = True