import Sofa


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SofaPython3')
    finger = rootNode.addChild('finger')
    finger.addObject('MeshVTKLoader', name='loader', filename='data/mesh/pneunetCutCoarse.vtk')
    finger.addObject('MeshTopology', src='@loader', name='container')
    finger.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1)
