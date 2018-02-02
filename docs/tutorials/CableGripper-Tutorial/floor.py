def createAndAddToNode(rootNode, name="thePlane",
                       rotation="0 0 270", translation="38 0 0", scale=10):

    planeNode = rootNode.createChild(name)
    planeNode.createObject('MeshObjLoader', name='loader',
                            filename="mesh/floorFlat.obj", triangulate="true",
                            rotation=rotation, scale=scale, translation=translation)

    planeNode.createObject('Mesh', src="@loader")
    planeNode.createObject('MechanicalObject', src="@loader")
    planeNode.createObject('Triangle',simulated="0", moving="0")
    planeNode.createObject('Line',simulated="0", moving="0")
    planeNode.createObject('Point',simulated="0", moving="0")
    planeNode.createObject('OglModel',name="Visual", fileMesh="mesh/floorFlat.obj",
                            color="1 0 0 1",
                            rotation=rotation, scale=scale, translation=translation)

    return planeNode
