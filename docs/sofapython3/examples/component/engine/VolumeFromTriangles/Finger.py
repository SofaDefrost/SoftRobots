# -*- coding: utf-8 -*-

import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject("RequiredPlugin", name='SofaPython3')
    rootNode.addObject('RequiredPlugin', pluginName=[
                        "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader
                        "Sofa.Component.LinearSolver.Iterative",  # Needed to use components CGLinearSolver
                        "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                        "Sofa.Component.Setting",  # Needed to use components BackgroundSetting
                        "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                        "Sofa.Component.Visual",  # Needed to use components VisualStyle
                        "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel, OglSceneFrame
                    ])

    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('DefaultAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    ##########################################
    # FEM Model                              #
    ##########################################
    finger = rootNode.addChild('finger')
    finger.addObject('EulerImplicitSolver')
    finger.addObject('CGLinearSolver', iterations=25, tolerance=1e-5, threshold=1e-5)

    finger.addObject('MeshSTLLoader', name='loader', filename=path + 'finger.stl')
    finger.addObject('MeshTopology', src='@loader', name="mesh")

    # Add a mechanicaobject component to stores the DoFs of the model
    finger.addObject('MechanicalObject', name='tetras', template='Vec3')
    finger.addObject('VolumeFromTriangles')

    ##########################################
    # Visualization                          #
    ##########################################
    fingerVisu = finger.addChild('visu')
    fingerVisu.addObject('MeshSTLLoader', filename=path + "finger.stl", name="loader")
    fingerVisu.addObject('OglModel', src="@loader", color=[0.0, 0.7, 0.7, 0.5])
    fingerVisu.addObject('BarycentricMapping')

    return rootNode
