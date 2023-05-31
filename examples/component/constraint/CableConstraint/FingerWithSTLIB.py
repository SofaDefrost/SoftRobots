# -*- coding: utf-8 -*-

import Sofa
import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'

from stlib3.scene import Scene
# A prefab that implements an ElasticMaterialObject
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.visuals import VisualModel
from FingerController import FingerController


def createScene(rootNode):
    rootNode.addObject("RequiredPlugin", name='SoftRobots')
    rootNode.addObject("RequiredPlugin", name='SofaPython3')
    rootNode.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                            "Sofa.Component.Constraint.Lagrangian.Correction",
                            # Needed to use components LinearSolverConstraintCorrection
                            "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                            "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                            "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader, MeshVTKLoader
                            "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                            "Sofa.Component.Mass",  # Needed to use components UniformMass
                            "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                            "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                            "Sofa.Component.Topology.Container.Dynamic",  # Needed to use components TetrahedronSetTopologyContainer
                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                            "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
                        ])
    scene = Scene(rootNode, gravity=[0.0, -9810.0, 0.0], dt=0.01)
    scene.VisualStyle.displayFlags = 'showBehavior'

    rootNode.addObject("FreeMotionAnimationLoop")
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject("GenericConstraintSolver", maxIterations=1000, tolerance=0.001)

    finger = ElasticMaterialObject(name="finger",
                                   volumeMeshFileName="mesh/finger.vtk",
                                   poissonRatio=0.45,
                                   youngModulus=600,
                                   totalMass=0.05)
    rootNode.addChild(finger)

    # To facilitate the selection of DoFs, SOFA has a concept called ROI (Region of Interest).
    # The idea is that ROI component "select" all DoFS that are enclosed by their "region".
    # We use ROI here to select a group of finger's DoFs that will be constrained to stay
    # at a fixed position.
    # You can either use "BoxROI"...
    finger.addObject('BoxROI', name='ROI1', box=[-15, 0, 0, 5, 10, 15], drawBoxes=True)
    # Or "SphereROI"...
    # finger.addObject('SphereROI', name='ROI', centers='0 0 0', radii='5')

    # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
    # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
    finger.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)

    ##########################################
    # Cable                                  #
    ##########################################

    #  This adds a new node in the scene. This node is appended to the finger's node.
    cable = finger.addChild('cable')

    # This adds a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
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

    # add a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last index is where the pullPoint is connected.
    cable.addObject('CableConstraint', name="aCableActuator",
                    indices=list(range(0, 14)),
                    pullPoint=[0.0, 12.5, 2.5])

    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bidirectional link
    # between the cable's DoFs and the finger's one's so that movements of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.addObject('BarycentricMapping')

    # This adds a PythonScriptController that permits to programmatically implement new behavior
    # or interactions using the Python programming language. The controller is referring to a
    # file named "controller.py".
    cable.addObject(FingerController(node=cable))

    ##########################################
    # Visualization                          #
    ##########################################
    # In Sofa, visualization is handled by adding a rendering model.
    finger.addChild(VisualModel(visualMeshPath=path + "finger.stl", color=[0.0, 0.7, 0.7, 1.0]))
    finger.VisualModel.addObject('BarycentricMapping', name='mapping')

    return rootNode
