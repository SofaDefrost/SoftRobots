import Sofa


def createScene(rootNode):
    rootNode.addObject('VisualStyle', displayFlags='showForceFields')
    rootNode.addObject('RequiredPlugin',
                       pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='SofaSparseSolver')
    rootNode.findData('gravity').value = [-9810, 0, 0];

    finger = rootNode.addChild('finger')
    finger.addObject('EulerImplicitSolver', name='odesolver')
    finger.addObject('SparseLDLSolver', name='directSolver')
    finger.addObject('MeshVTKLoader', name='loader', filename='data/mesh/pneunetCutCoarse.vtk')
    finger.addObject('MeshTopology', src='@loader', name='container')
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1)
    finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=500)
    finger.addObject('UniformMass', totalMass=0.0008)
    finger.addObject('BoxROI', name='boxROISubTopo', box=[-100, 22.5, -8, -19, 28, 8], strict=False)
    finger.addObject('BoxROI', name='boxROI', box=[-10, 0, -20, 0, 30, 20], drawBoxes=True)
    finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12, angularStiffness=1e12)

    modelSubTopo = finger.addChild('modelSubTopo')
    modelSubTopo.addObject('MeshTopology', position='@loader.position', tetrahedra='@boxROISubTopo.tetrahedraInROI',
                           name='container')
    modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                           youngModulus=1500)
