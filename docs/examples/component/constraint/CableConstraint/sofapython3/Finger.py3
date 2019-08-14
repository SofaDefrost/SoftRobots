# -*- coding: utf-8 -*-

import Sofa
import Sofa.Core

import os

path = os.path.dirname(os.path.abspath(__file__))+'../mesh/'
from FingerController import FingerController

def createScene(rootNode):
                rootNode.addObject('RequiredPlugin', pluginName='SoftRobots')
                rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

                rootNode.addObject('FreeMotionAnimationLoop')

                # Add a QPInverseProblemSolver to the scene if you need to solve inverse problem like the one that involved
                # when manipulating the robots by specifying their effector's position instead of by direct control
                # of the actuator's parameters.
                #rootNode.addObject('QPInverseProblemSolver', printLog='1')
                # Otherwise use a GenericConstraintSolver
                rootNode.addObject('GenericConstraintSolver', tolerance="1e-5", maxIterations="100")

                rootNode.gravity = [0, -9810, 0]
                rootNode.dt=0.01
                rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
                rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

                ##########################################
                # FEM Model                              #
                ##########################################
                finger = rootNode.addChild('finger')
                finger.addObject('EulerImplicit', name='odesolver', firstOrder='0', rayleighMass="0.1", rayleighStiffness="0.1")
                finger.addObject('SparseLDLSolver', name='preconditioner')

		        # Add a componant to load a VTK tetrahedral mesh and expose the resulting topology in the scene .
                finger.addObject('MeshVTKLoader', name='loader', filename=path+'finger.vtk')
                finger.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                finger.addObject('TetrahedronSetTopologyModifier')
                finger.addObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
                finger.addObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')

                # Create a mechanicaobject component to stores the DoFs of the model
                finger.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5', rx='0', dz='0')

                # Gives a mass to the model
                finger.addObject('UniformMass', totalMass='0.075')

                # Add a TetrahedronFEMForceField componant which implement an elastic material model solved using the Finite Element Method on
                # tetrahedrons.
                finger.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.45',  youngModulus='600')

                # To facilitate the selection of DoFs, SOFA has a concept called ROI (Region of Interest).
		        # The idea is that ROI component "select" all DoFS that are enclosed by their "region".
                # We use ROI here to select a group of finger's DoFs that will be constrained to stay
                # at a fixed position.
                # You can either use "BoxROI"...
                finger.addObject('BoxROI', name='ROI1', box='-15 0 0 5 10 15', drawBoxes='true')
                # Or "SphereROI"...
                #finger.addObject('SphereROI', name='ROI', centers='0 0 0', radii='5')

                # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
                # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
                finger.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness='1e12')

                # It is also possible to simply set by hand the indices of the points you want to fix.
                #finger.addObject('RestShapeSpringsForceField', points='0 1 2 11 55', stiffness='1e12')

                finger.addObject('LinearSolverConstraintCorrection')


                ##########################################
                # Cable                                  #
                ##########################################

                #  This create a new node in the scene. This node is appended to the finger's node.
                cable = finger.addChild('cable')

                # This create a MechanicalObject, a componant holding the degree of freedom of our
                # mechanical modelling. In the case of a cable it is a set of positions specifying
                # the points where the cable is passing by.
                cable.addObject('MechanicalObject',
                        position=(
                                "-17.5 12.5 2.5 " +
                                "-32.5 12.5 2.5 " +
                                "-47.5 12.5 2.5 " +
                                "-62.5 12.5 2.5 " +
                                "-77.5 12.5 2.5 " +

                                "-83.5 12.5 4.5 " +
                                "-85.5 12.5 6.5 " +
                                "-85.5 12.5 8.5 " +
                                "-83.5 12.5 10.5 " +

                                "-77.5 12.5 12.5 " +
                                "-62.5 12.5 12.5 " +
                                "-47.5 12.5 12.5 " +
                                "-32.5 12.5 12.5 " +
                                "-17.5 12.5 12.5 " ))

                # Create a CableConstraint object with a name.
                # the indices are referring to the MechanicalObject's positions.
                # The last indice is where the pullPoint is connected.
                cable.addObject('CableConstraint', name="aCableActuator",
                        #indices=range(0,14),
                        indices="0 1 2 3 4 5 6 7 8 9 10 11 12 13",
                        pullPoint="0.0 12.5 2.5")

                # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
                # between the cable's DoFs and the finger's ones so that movements of the cable's DoFs will be mapped
                # to the finger and vice-versa;
                cable.addObject('BarycentricMapping')

                # This create a PythonScriptController that permits to programatically implement new behavior
                # or interactions using the Python programming langage. The controller is referring to a
                # file named "controller.py".
                cable.addObject(FingerController(node=cable))

                ##########################################
                # Visualization                          #
                ##########################################
                # In Sofa, visualization is handled by adding a rendering model.
                # Create an empty child node to store this rendering model.
                fingerVisu = finger.addChild('visu')

                # Add to this empty node a rendering model made of triangles and loaded from an stl file.
                fingerVisu.addObject('MeshSTLLoader', filename=path+"finger.stl", name="loader")
                fingerVisu.addObject('OglModel', src="@loader", template='ExtVec3f', color="0.0 0.7 0.7")

                # Add a BarycentricMapping to deform the rendering model in a way that follow the ones of the parent mechanical model.
                fingerVisu.addObject('BarycentricMapping')

                return rootNode
