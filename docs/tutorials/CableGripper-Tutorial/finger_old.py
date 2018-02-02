import os
path = os.path.dirname(os.path.abspath(__file__))+'/data/mesh/'

def createAndAddToNode(rootNode, name="theFinger", rotation="0 0 0", translation="0 0 0",
                       fixingbox='135 0 0 155 10 15', pullpoint="150 12.5 2.5"):
	finger = rootNode.createChild(name)
	finger.findData('activated').value = 1
	finger.createObject('EulerImplicit', name='odesolver')
	finger.createObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', 
						       tolerance='1e-10', preconditioners='preconditioner', 
						       use_precond='true', update_step='1')

	finger.createObject('MeshVTKLoader', name='loader', filename=path+'finger.vtk', rotation=rotation, translation=translation)
	finger.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
	finger.createObject('TetrahedronSetTopologyModifier')
	finger.createObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
	finger.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')

	finger.createObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5', rx='0', dz='0')
	finger.createObject('UniformMass', totalmass='0.5')
	finger.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus='18000')

	finger.createObject('BoxROI', name='boxROI', box=fixingbox, drawBoxes='true', doUpdate="0")
	finger.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e8', springColor='1 0 0 1')

	finger.createObject('SparseLDLSolver', name='preconditioner')
	finger.createObject('LinearSolverConstraintCorrection', solverName='preconditioner')

	#finger/actuator
	actuator = finger.createChild('actuator')
	actuator.createObject('MechanicalObject', name="actuatorPoint", 
				rotation=rotation, translation=translation,
				position=("-103.071  7.36479  7.14143" +
				" -17.5 12.5 2.5 " +
				" -32.5 12.5 2.5 " +
				" -47.5 12.5 2.5 " +
				" -62.5 12.5 2.5 " +
				" -77.5 12.5 2.5 " +

				" -83.5 12.5 4.5 " +
				" -85.5 12.5 6.5 " +
				" -85.5 12.5 8.5 " +
				" -83.5 12.5 10.5 " +

				" -77.5 12.5 12.5 " +
				" -62.5 12.5 12.5 " +
				" -47.5 12.5 12.5 " +
				" -32.5 12.5 12.5 " +
				" -17.5 12.5 12.5 " ))

	actuator.createObject('CableConstraint', template='Vec3d', name='cc',
				indices='1 2 3 4 5 6 7 8 9 10 11 12 13 14', pullPoint=pullpoint,
				valueIndex="0", constraintIndex='0')
	actuator.createObject('BarycentricMapping', mapForces="false", mapMasses="false")

	#finger/fingerVisu
	fingerVisu = finger.createChild('visu')
	fingerVisu.createObject('OglModel', filename=path+"finger.stl", template='ExtVec3f', color="0.0 0.7 0.7", 
				rotation=rotation, translation=translation)
	fingerVisu.createObject('BarycentricMapping')

	#finger/fingerCollis
	fingerCollis = finger.createChild('collis')
	fingerCollis.createObject('MeshSTLLoader', name='loader', filename=path+'finger.stl', rotation=rotation, translation=translation)
	fingerCollis.createObject('TriangleSetTopologyContainer', src='@loader', name='container')
	fingerCollis.createObject('MechanicalObject', template='Vec3d')
	fingerCollis.createObject('Triangle')
	fingerCollis.createObject('Line')
	fingerCollis.createObject('Point')
	fingerCollis.createObject('BarycentricMapping')
	
	return finger
	
	
