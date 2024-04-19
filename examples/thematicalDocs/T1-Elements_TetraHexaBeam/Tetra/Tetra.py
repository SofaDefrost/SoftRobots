import Sofa

import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


# Units: cm and kg

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels showForceFields')

    rootNode.dt = 0.001
    rootNode.gravity = [0., 0., -9810]

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    ###############################
    # MECHANICAL MODEL
    ###############################

    robot = rootNode.addChild('robot')
    robot.addObject('EulerImplicitSolver')
    robot.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')
    robot.addObject('MeshVTKLoader', name='loader', filename=path + 'branch.vtu')
    # Here you set the tetrahedra topology
    robot.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@loader.tetrahedra',
                    name='container', createTriangleArray=1)
    robot.addObject('TetrahedronSetTopologyModifier')

    robot.addObject('MechanicalObject', name='tetras')
    robot.addObject('UniformMass', totalMass=0.4)
    robot.addObject('TetrahedronFEMForceField', poissonRatio=0.45, youngModulus=600)

    robot.addObject('BoxROI', name='boxROI',
                    box=[[-30, 80, 70, 30, 140, 130], [-130, -100, 70, -60, -20, 130], [130, -100, 70, 60, -20, 130]],
                    drawBoxes=True)
    robot.addObject('FixedProjectiveConstraint', indices="@boxROI.indices")

    return rootNode
