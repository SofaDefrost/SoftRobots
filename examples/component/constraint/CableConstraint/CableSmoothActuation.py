# -*- coding: utf-8 -*-
__authors__ = "tnavez"

import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', pluginName= ['SoftRobots', 
        'Sofa.Component.SolidMechanics.Spring', 
        'Sofa.Component.Engine.Select', 
        'Sofa.Component.ODESolver.Backward',
        'Sofa.Component.IO.Mesh',
        'Sofa.GL.Component.Rendering2D',
        'Sofa.GL.Component.Rendering3D',
        'Sofa.GL.Component.Shader',
        'Sofa.Component.LinearSolver.Preconditioner',
        'Sofa.Component.LinearSolver.Iterative',
        'Sofa.Component.Diffusion',
        'Sofa.Component.SolidMechanics.FEM.Elastic',
        'Sofa.Component.LinearSolver.Direct',
        'Sofa.Component.Topology.Mapping',
        'Sofa.Component.AnimationLoop',
        'Sofa.Component.Constraint.Lagrangian.Correction',
        'Sofa.Component.Constraint.Lagrangian.Solver',
        'Sofa.Component.Mass',
        'Sofa.Component.StateContainer',
        'Sofa.Component.Topology.Container.Constant',
        'Sofa.Component.Topology.Container.Dynamic',
        'Sofa.Component.Visual',
        'Sofa.Component.Mapping.Linear'])

    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', printLog='0')

    ################################################################################################################
    ################################################### Bunny ######################################################
    ################################################################################################################
    bunny = rootNode.addChild('bunny')
    bunny.addObject('EulerImplicitSolver', name='odesolver')
    bunny.addObject('EigenSimplicialLDLT', template="CompressedRowSparseMatrixMat3x3d")

    bunny.addObject('MeshVTKLoader', name='loader', filename=path+'Hollow_Stanford_Bunny.vtu')
    bunny.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    bunny.addObject('TetrahedronSetTopologyModifier')

    bunny.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale='4e-5', rx='0', dz='0')
    bunny.addObject('UniformMass', totalMass='0.5')
    bunny.addObject('FastTetrahedralCorotationalForceField', template='Vec3', name='FEM', method='large', poissonRatio='0.3',  youngModulus='10000')

    bunny.addObject('BoxROI', name='boxROI', box=[-5, -5.0, -5,  5, -4.5, 5], drawBoxes=True)
    bunny.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')
    
    bunny.addObject('GenericConstraintCorrection')
    
    

	################################################################################################################
    ################################################ Constraints ###################################################
    ################################################################################################################    
    constraints = bunny.addChild('Constraints')
    constraints.addObject('MeshSTLLoader', name='loaderSufaceCable', filename=path+'Bunny.stl')
    constraints.addObject('MeshTopology', src='@loaderSufaceCable', name='topoSufaceCable')
    constraints.addObject('MechanicalObject')
    
    constraints.addObject('CableConstraint', template='Vec3', name='CableSphericalSurface', pullPoint= [-4.4, -10, 3.1], minForce=0, 
                         value = 1, centers = [[-4.5, -2.54, 0.0]],
                         surfaceTopology = "@topoSufaceCable",
                         method="geodesic", radii = [1.0], # 0.8
                         drawPoints = True, drawPulledAreas = True) 
    
    constraints.addObject('BarycentricMapping')    
    

	################################################################################################################
    ############################################### Visualization ##################################################
    ################################################################################################################
    bunnyVisu = bunny.addChild('visu')
    bunnyVisu.addObject('TriangleSetTopologyContainer', name='container')
    bunnyVisu.addObject('TriangleSetTopologyModifier')
    bunnyVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping')
    bunnyVisu.addObject('OglModel', template='Vec3d', color='0.7 0.4 0.4 1')
    bunnyVisu.addObject('IdentityMapping')

    return rootNode


