import os
from math import pi

dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    rootNode.addObject('RequiredPlugin', name='ArticulatedSystemPlugin')  # Needed to use components [ArticulatedHierarchyContainer,ArticulatedSystemMapping,Articulation,ArticulationCenter]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [UncoupledConstraintCorrection]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver')  # Needed to use components [GenericConstraintSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')  # Needed to use components [FixedProjectiveConstraint]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Generate')  # Needed to use components [GenerateRigidMass]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')  # Needed to use components [MeshSTLLoader]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')  # Needed to use components [RigidMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting')  # Needed to use components [BackgroundSetting]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [BeamFEMForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]  
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')  # Needed to use components [OglModel] 

    rootNode.dt = 0.01
    rootNode.gravity = [0., -9810., 0.]
    rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=500, tolerance=1e-4)

    # Simulation node
    simulation = rootNode.addChild('Simulation')
    simulation.addObject('EulerImplicitSolver')
    simulation.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
    simulation.addObject('GenericConstraintCorrection')

    object = simulation.addChild('Object')
    object.addObject('MechanicalObject',  template='Vec1', position=0.)
    object.addObject("RestShapeSpringsForceField", points=[0], stiffness=1e1)
    # Try to change the data field "value" from the GUI
    object.addObject('JointConstraint', template='Vec1', index=0, value=0, valueType="displacement",
                     minDisplacement=-pi, maxDisplacement=pi)

    rigid = object.addChild('Rigid')
    rigid.addObject('MechanicalObject', template='Rigid3', position=[[0., 0., 0., 0., 0., 0., 1.]]*2)
    rigid.addObject('UniformMass', totalMass=0.003)
    rigid.addObject('ArticulatedSystemMapping', 
                    input1=object.getMechanicalState().linkpath, 
                    output=rigid.getMechanicalState().linkpath)

    visual = rigid.addChild('VisualModel')
    visual.addObject('MeshSTLLoader', name='loader', filename=dirPath + 'mesh/arm.stl', translation=[-45, 0., 0.])
    visual.addObject('MeshTopology', src='@loader')
    visual.addObject('OglModel', color=[0.9, 0.9, 0.9, 1.])
    visual.addObject('RigidMapping', index=1)

    articulationCenters = object.addChild('Articulation')
    articulationCenter = articulationCenters.addChild('ArticulationCenter')
    articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1,
                                 posOnParent=[0., 0., 0.],
                                 posOnChild=[-45, 0., 0.], articulationProcess=1)
    articulation = articulationCenter.addChild('Articulations')
    articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 0, 1],
                           articulationIndex=0)

    object.addObject('ArticulatedHierarchyContainer')

    return rootNode
