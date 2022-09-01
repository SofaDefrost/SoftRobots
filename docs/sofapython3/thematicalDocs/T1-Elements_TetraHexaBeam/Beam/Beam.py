import Sofa
from math import sin, cos


# Units : mm and kg

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SofaPython3')

    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    rootNode.dt = 0.001
    rootNode.gravity = [0., 0., -9810]
    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    # Compute the beam topology using python
    length = 100
    nbEdges = 10
    edgesList = []
    for i in range(0, nbEdges):
        edgesList.append([i, i + 1])

    positionsList = []
    for i in range(0, nbEdges + 1):
        dx = length / (nbEdges + 1)
        positionsList.append([dx * i, 0, 0, 0, 0, 0, 1])

    # Beam model
    beam = rootNode.addChild('beam')
    beam.addObject('EulerImplicitSolver')
    beam.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixd')
    beam.addObject('MeshTopology', edges=edgesList)
    beam.addObject('MechanicalObject', template="Rigid3", position=positionsList, name='frame')
    beam.addObject('BeamInterpolation', name='interpol', crossSectionShape='rectangular', lengthY=5, lengthZ=5,
                   defaultYoungModulus=1.8e6)
    beam.addObject('AdaptiveBeamForceFieldAndMass', name="BeamForceField", computeMass=1, massDensity=0.001)
    beam.addObject('RestShapeSpringsForceField', points=0, stiffness=100e10, angularStiffness=100e10)

    # Visualization
    visuNode = beam.addChild('visualization')
    visuNode.addObject('RegularGridTopology', name='topology', n=[20, 5, 5], min=[0, -2.5, -2.5], max=[100, 2.5, 2.5])
    visuNode.addObject('MechanicalObject', name='test', src='@topology')
    visuNode.addObject('AdaptiveBeamMapping', interpolation="@../interpol", input="@../frame", output="@test")

    oglNode = visuNode.addChild('ogl')
    oglNode.addObject('OglModel', color=[0, 1, 0, 1])
    oglNode.addObject('IdentityMapping')

    return rootNode
