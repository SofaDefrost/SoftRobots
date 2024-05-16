# -*- coding: utf-8 -*-

import Sofa

import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
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
        "Sofa.Component.Setting",  # Needed to use components BackgroundSetting
        "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
        "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
        "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
        "Sofa.Component.Visual",  # Needed to use components VisualStyle
        "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
    ])
    rootNode.addObject('VisualStyle', displayFlags="showVisualModels hideBehaviorModels hideCollisionModels \
                        hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe")

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-3)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.findData('gravity').value = [0, 0, -981.0]
    rootNode.findData('dt').value = 0.01

    ##########################################
    # FEM Model                              #
    ##########################################
    accordion = rootNode.addChild('accordion')
    accordion.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=0.2, rayleighMass=0.2)
    accordion.addObject('SparseLDLSolver')

    accordion.addObject('MeshVTKLoader', name='loader', filename=path + 'Springy.vtk', rotation=[0, 0, 0])
    accordion.addObject('MeshTopology', src='@loader')

    accordion.addObject('MechanicalObject', name='tetras', template='Vec3')
    accordion.addObject('UniformMass', totalMass=0.030)
    accordion.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                        youngModulus=500)

    accordion.addObject('BoxROI', name='ROI1', box=[-2, -2, 0, 2, 2, 0.5], drawBoxes=True)
    accordion.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)
    # accordion.addObject('FixedProjectiveConstraint', indices='@ROI1.indices')

    accordion.addObject('LinearSolverConstraintCorrection')

    ##########################################
    # Pressure                               #
    ##########################################
    #  This add a new node in the scene. This node is appended to the accordion's node.
    cavity = accordion.addChild('cavity')

    #  This adds a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a pneumatic actuation it is a set of positions describing the cavity wall.
    cavity.addObject('MeshSTLLoader', name='loader', filename=path + 'Springy_Cavity.stl')
    cavity.addObject('MeshTopology', src='@loader', name='topo')
    cavity.addObject('MechanicalObject', name='cavity')

    # Add a SurfacePressureConstraint object with a name.
    cavity.addObject('SurfacePressureConstraint', template='Vec3', name="pressure",
                     triangles='@topo.triangles',
                     valueType=1,
                     value=8)

    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    #  between the cavity wall (surfacic mesh) and the accordion (volumetric mesh) so that movements of the cavity's DoFs will be mapped
    #  to the accordion and vice-versa;
    cavity.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    ##########################################
    # Visualization                          #
    ##########################################
    accordionVisu = accordion.addChild('visu')
    accordionVisu.addObject('MeshSTLLoader', filename=path + "Springy.stl", name="loader")
    accordionVisu.addObject('OglModel', src="@loader", color=[0.4, 0.4, 0.4, 0.5])
    accordionVisu.addObject('BarycentricMapping')

    return rootNode
