# -*- coding: utf-8 -*-

import os
from FingerController import FingerController
path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject("RequiredPlugin", name='SoftRobots')
    rootNode.addObject("RequiredPlugin", name='SofaPython3')
    rootNode.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                            "Sofa.Component.Constraint.Lagrangian.Correction",  # Needed to use components GenericConstraintCorrection
                            "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                            "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                            "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader, MeshVTKLoader
                            "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                            "Sofa.Component.Mass",  # Needed to use components UniformMass
                            "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                            "Sofa.Component.Setting",  # Needed to use components BackgroundSetting
                            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                            "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                            "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                            "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel, OglSceneFrame
                        ])
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    # Add a QPInverseProblemSolver to the scene if you need to solve inverse problem like the one that involved
    # when manipulating the robots by specifying their effector's position instead of by direct control
    #  of the actuator's parameters.
    # rootNode.addObject('QPInverseProblemSolver', printLog=False)
    # Otherwise use a GenericConstraintSolver
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=100)

    rootNode.gravity = [0, -9810, 0]
    rootNode.dt = 0.01

    ##########################################
    # FEM Model                              #
    ##########################################
    finger = rootNode.addChild('finger')
    finger.addObject('EulerImplicitSolver', name='odesolver', rayleighMass=0.1, rayleighStiffness=0.1)
    finger.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')

    # Add a component to load a VTK tetrahedral mesh and expose the resulting topology in the scene .
    finger.addObject('MeshVTKLoader', name='loader', filename=path + 'finger.vtk')
    finger.addObject('MeshTopology', src='@loader', name='container')

    # Create a MechanicaObject component to stores the DoFs of the model
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5)

    # Gives a mass to the model
    finger.addObject('UniformMass', totalMass=0.075)

    # Add a TetrahedronFEMForceField component which implement an elastic material model solved using the Finite Element Method on
    #  tetrahedrons.
    finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.45,
                     youngModulus=600)

    # To facilitate the selection of DoFs, SOFA has a concept called ROI (Region of Interest).
    #  The idea is that ROI component "select" all DoFS that are enclosed by their "region".
    # We use ROI here to select a group of finger's DoFs that will be constrained to stay
    # at a fixed position.
    # You can either use 'BoxROI'...
    finger.addObject('BoxROI', name='roi', box=[-15, 0, 0, 5, 10, 15], drawBoxes=True)
    # Or 'SphereROI'...
    # finger.addObject('SphereROI', name='roi', centers=[0, 0, 0], radii=5)

    # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
    # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
    finger.addObject('RestShapeSpringsForceField', points=finger.roi.indices.getLinkPath(), stiffness=1e12)

    # It is also possible to simply set by hand the indices of the points you want to fix.
    # finger.addObject('RestShapeSpringsForceField', points=[0, 1, 2, 11, 55], stiffness=1e12)

    finger.addObject('GenericConstraintCorrection')

    ##########################################
    # Cable                                  #
    ##########################################

    #  This creates a new node in the scene. This node is appended to the finger's node.
    cable = finger.addChild('cable')

    #  This creates a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    #  the points where the cable is passing by.
    cable.addObject('MechanicalObject',
                    position=[
                        [-17.5, 12.5, 2.5],
                        [-32.5, 12.5, 2.5],
                        [-47.5, 12.5, 2.5],
                        [-62.5, 12.5, 2.5],
                        [-77.5, 12.5, 2.5],

                        [-85.5, 12.5, 6.5],
                        [-85.5, 12.5, 8.5],
                        [-83.5, 12.5, 4.5],
                        [-83.5, 12.5, 10.5],

                        [-77.5, 12.5, 12.5],
                        [-62.5, 12.5, 12.5],
                        [-47.5, 12.5, 12.5],
                        [-32.5, 12.5, 12.5],
                        [-17.5, 12.5, 12.5]])

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last index is where the pullPoint is connected.
    cable.addObject('CableConstraint', name="aCableActuator",
                    indices=list(range(0, 14)),
                    # indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
                    minForce=0,  # Set that the cable can't push
                    pullPoint=[0.0, 12.5, 2.5])

    # This creates a BarycentricMapping. A BarycentricMapping is a key element as it will create a bidirectional link
    #  between the cable's DoFs and the finger's one's so that movements of the cable's DoFs will be mapped
    #  to the finger and vice-versa;
    cable.addObject('BarycentricMapping')

    # This creates a PythonScriptController that permits to programmatically implement new behavior
    #  or interactions using the Python programming language. The controller is referring to a
    #  file named "controller.py".
    cable.addObject(FingerController(name="FingerController", node=cable))

    ##########################################
    # Visualization                          #
    ##########################################
    # In Sofa, visualization is handled by adding a rendering model.
    #  Create an empty child node to store this rendering model.
    fingerVisu = finger.addChild('visu')

    # Add to this empty node a rendering model made of triangles and loaded from a stl file.
    fingerVisu.addObject('MeshSTLLoader', filename=path + "finger.stl", name="loader")
    fingerVisu.addObject('OglModel', src="@loader", color=[0.0, 0.7, 0.7, 1])

    # Add a BarycentricMapping to deform the rendering model in a way that follow the ones of the parent mechanical model.
    fingerVisu.addObject('BarycentricMapping')

    return rootNode
