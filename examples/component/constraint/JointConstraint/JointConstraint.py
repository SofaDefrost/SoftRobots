import os

dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')

    rootNode.dt = 0.1
    rootNode.gravity = [0., -981., 0.]
    rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels')
    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.])
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver')

    # Simulation node
    simulation = rootNode.addChild('Simulation')
    simulation.addObject('EulerImplicitSolver')
    simulation.addObject('CGLinearSolver')

    object = simulation.addChild('Object')
    object.addObject('MechanicalObject', name='dofs', template='Vec1', position=0.)
    # Try to change the imposed value from the GUI 
    object.addObject('JointConstraint', template='Vec1', index=0, imposedValue=2, valueType="displacement")

    rigid = object.addChild('Rigid')
    rigid.addObject('MechanicalObject', template='Rigid3', name='dofs',
                    position=[[0., 0., 0., 0., 0., 0., 1.], [4.5, 0., 0., 0., 0., 0., 1.]])
    rigid.addObject('BeamFEMForceField', name='FEM', radius=0.2, youngModulus=1e3, poissonRatio=0.45)
    rigid.addObject('MeshTopology', lines=[0, 1])

    visual = rigid.addChild('VisualModel')
    visual.addObject('MeshSTLLoader', name='loader', filename=dirPath + 'mesh/arm.stl', translation=[-4.5, 0., 0.],
                     scale3d=[0.1, 0.1, 0.1])
    visual.addObject('MeshTopology', src='@loader')
    visual.addObject('OglModel', color=[0.9, 0.9, 0.9, 1.])
    visual.addObject('RigidMapping', index=1)

    rigid.addObject('GenerateRigidMass', name='mass', density=0.002, src=visual.loader.getLinkPath())
    rigid.addObject('FixedConstraint', template='Rigid3', indices=0)
    rigid.addObject('ArticulatedSystemMapping', input1=object.dofs.getLinkPath(), output=rigid.dofs.getLinkPath())
    rigid.addObject('UniformMass', vertexMass=rigid.mass.findData('rigidMass').getLinkPath())

    object.addObject('UncoupledConstraintCorrection')
    object.addObject('ArticulatedHierarchyContainer')

    articulationCenters = object.addChild('Articulation')
    articulationCenter = articulationCenters.addChild('ArticulationCenter')
    articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.],
                                 posOnChild=[-4.5, 0., 0.], articulationProcess=0)
    articulation = articulationCenter.addChild('Articulations')
    articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 0, 1],
                           articulationIndex=0)

    return rootNode
