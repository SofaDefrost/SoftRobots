# -*- coding: utf-8 -*-

from ControllerDisplacement import ControllerDisplacement

import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')

    rootNode.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=100)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    ##########################################
    # FEM Model                              #
    ##########################################
    finger = rootNode.addChild('finger')
    finger.addObject('EulerImplicitSolver', name='odesolver', firstOrder=True)
    finger.addObject('SparseLDLSolver', name='preconditioner', template='CompressedRowSparseMatrixMat3x3d')

    # Add a component to load a VTK tetrahedral mesh and expose the resulting topology in the scene .
    finger.addObject('MeshVTKLoader', name='loader', filename=path + 'finger.vtk')
    finger.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetras="@loader.tetras", name='container')

    # add a mechanical object component to stores the DoFs of the model
    finger.addObject('MechanicalObject', name='tetras', template='Vec3')

    # Gives a mass to the model
    finger.addObject('UniformMass', totalMass=0.5)

    # Add a TetrahedronFEMForceField component which implements an elastic material model solved using the Finite Element Method on
    # tetrahedrons.
    finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=18000)

    # To facilitate the selection of DoFs, SOFA has a concept called ROI (Region of Interest).
    # The idea is that ROI component "select" all DoFS that are enclosed by their "region".
    # We use ROI here to select a group of finger's DoFs that will be constrained to stay
    # at a fixed position.
    # You can either use "BoxROI"...
    finger.addObject('BoxROI', name='ROI', box=[-15, 0, 0, 5, 10, 15], drawBoxes=True)
    # Or "SphereROI"...
    # finger.addObject('SphereROI', name='ROI', centers=[0, 0, 0], radii=5)

    # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
    # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
    finger.addObject('RestShapeSpringsForceField', points='@ROI.indices', stiffness=1e12)

    # It is also possible to simply set by hand the indices of the points you want to fix.
    # finger.addObject('RestShapeSpringsForceField', points=[0, 1, 2, 11, 55], stiffness=1e12)

    finger.addObject('LinearSolverConstraintCorrection')

    ##########################################
    # Cable                                  #
    ##########################################
    cable = finger.addChild('cable')
    cable.addObject('MechanicalObject',
                    position=[
                        [-17.5, 12.5, 2.5],
                        [-32.5, 12.5, 2.5],
                        [-47.5, 12.5, 2.5],
                        [-62.5, 12.5, 2.5],
                        [-77.5, 12.5, 2.5],

                        [-83.5, 12.5, 4.5],
                        [-85.5, 12.5, 6.5],
                        [-85.5, 12.5, 8.5],
                        [-83.5, 12.5, 10.5],

                        [-77.5, 12.5, 12.5],
                        [-62.5, 12.5, 12.5],
                        [-47.5, 12.5, 12.5],
                        [-32.5, 12.5, 12.5],
                        [-17.5, 12.5, 12.5]])

    # Add a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last index is where the pullPoint is connected.
    # By default, the Cable is controlled by displacement, rather than force.
    cable.addObject('CableConstraint', name="aCable",
                    indices=list(range(14)),
                    pullPoint=[0.0, 12.5, 2.5])

    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bidirectional link
    # between the cable's DoFs and the finger's ones so that movements of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.addObject('BarycentricMapping')

    # This adds a PythonScriptController that permits to programmatically implement new behavior
    # or interactions using the Python programming language. The controller is referring to a
    # file named "controller.py".
    cable.addObject(ControllerDisplacement(node=cable))

    ##########################################
    # Visualization                          #
    ##########################################
    # In Sofa, visualization is handled by adding a rendering model.
    # add an empty child node to store this rendering model.
    fingerVisu = finger.addChild('visu')

    # Add to this empty node a rendering model made of triangles and loaded from a stl file.
    fingerVisu.addObject('MeshSTLLoader', name='loader', filename=path + "finger.stl")
    fingerVisu.addObject('OglModel', src='@loader', color=[0.0, 0.7, 0.7, 1])

    # Add a BarycentricMapping to deform the rendering model to follow the ones of the mechanical model.
    fingerVisu.addObject('BarycentricMapping')

    return rootNode
