def createAndAddToNode(rootNode, name="theCube"):
	################################ Grasped Object ###################################
	# mechanics
	cube =rootNode.createChild(name)
	cube.createObject('EulerImplicit', name='odesolver')
	cube.createObject('CGLinearSolver', name='linearSolver')
	cube.createObject('MechanicalObject', template="Rigid", scale="6", dx="67.0", dy="10", dz="8", rx="10" ,ry="10")
	cube.createObject('UniformMass', mass='0.03 10 1000 0 0 0 1000 0 0 0 1000')
	cube.createObject('UncoupledConstraintCorrection')

	#collision
	cubeCollis = cube.createChild('cubeCollis')
	cubeCollis.createObject('MeshObjLoader', name="loader", filename="mesh/smCube27.obj", triangulate="true",  scale="6")
	cubeCollis.createObject('Mesh', src="@loader")
	cubeCollis.createObject('MechanicalObject')
	cubeCollis.createObject('Triangle')
	cubeCollis.createObject('Line')
	cubeCollis.createObject('Point')
	cubeCollis.createObject('RigidMapping')

	#visualization
	cubeVisu = cube.createChild('cubeVisu')
	cubeVisu.createObject('OglModel', name="Visual", fileMesh="mesh/smCube27.obj", color="0.0 0.1 0.5", scale="6.2")
	cubeVisu.createObject('RigidMapping')

        return cube


