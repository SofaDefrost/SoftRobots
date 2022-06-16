# -*- coding: utf-8 -*-

import Sofa

import os

path = os.path.dirname(os.path.abspath(__file__)) + '/'
pathMesh = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject("RequiredPlugin", name='SofaPython3')
    rootNode.addObject('RequiredPlugin', pluginName=[
        "Sofa.Component.IO.Mesh",  # Needed to use components MeshOBJLoader
        "Sofa.Component.LinearSolver.Iterative",  # Needed to use components CGLinearSolver
        "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
        "Sofa.Component.Setting",  # Needed to use components BackgroundSetting
        "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel, OglSceneFrame
    ])

    rootNode.addObject('DefaultAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    point = rootNode.addChild('point')
    point.addObject('EulerImplicitSolver', firstOrder=True)
    point.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    point.addObject('MechanicalObject', template='Rigid3',
                    position=[0, 0, 0, 0, 0, 0, 1],
                    showObject=True,
                    showObjectScale=0.1,
                    drawMode=1,
                    showColor=[255, 255, 255, 255])
    # The AnimationEditor takes multiple options
    # template : should be the same as the mechanical you want to animate
    # filename : file in which the animation will be saved
    # load : set to true to load the animation at init (default is true)
    # loop : when the animation is playing, set this option to true to loop and start again the animation
    # dx : to control the animation in displacement instead of time
    # frameTime (default is 0.01)
    # drawTimeline (default is true)
    # drawTrajectory (default is true)
    # drawSize : coefficient size of displayed elements of trajectory
    point.addObject('AnimationEditor', name='animation',
                    template='Rigid3', filename=path + 'RigidAnimation.txt',
                    load=True,
                    drawTimeline=True, drawTrajectory=True)

    visu = point.addChild('visu')
    visu.addObject('MeshOBJLoader', name='loader', filename='mesh/cube.obj')
    visu.addObject('OglModel', src='@loader', filename='mesh/cube.obj')
    visu.addObject('RigidMapping')

    return rootNode
