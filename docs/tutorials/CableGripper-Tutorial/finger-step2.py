import os
path = os.path.dirname(os.path.abspath(__file__))+'/data/mesh/'

def createAndAddToNode(rootNode, name="theFinger", rotation="0 0 0", translation="0 0 0",
                       fixingbox='135 0 0 155 10 15', pullpoint="150 12.5 2.5"):
	finger = rootNode.createChild(name)
        finger.createObject('EulerImplicit', name='odesolver', firstOrder='1')
        finger.createObject('SparseLDLSolver', name='preconditioner')

        finger.createObject('MeshVTKLoader', name='loader', filename=path+'finger.vtk')
        finger.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
	finger.createObject('MechanicalObject', name='tetras', template='Vec3d')

        ## To be properly simulated and to interact with gravity or inertia forces, an object
        ## also needs a mass. You can add a given mass with a uniform distribution for an object
        ## by adding a UniformMass component to the finger node
        finger.createObject('UniformMass', totalmass=' 0.5')

        ## The next component to add is a FEM forcefield which defines how the object reacts
        ## to a loading (i.e. which deformations are created from forces applied onto it).
	## Here, because the finger is made of silicone, its mechanical behavior is assumed elastic.
        ## This behavior is available via the TetrahedronFEMForceField component.
	finger.createObject('TetrahedronFEMForceField', template='Vec3d',
                            name='FEM', method='large', poissonRatio='0.3',  youngModulus='18000')

	## Set the ROI of points of the model to fix.
	## You can either use "BoxROI"...
	finger.createObject('BoxROI', name='ROI1', box='-15 0 0 5 10 15', drawBoxes='true')
	finger.createObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness='1e12')

	finger.createObject('LinearSolverConstraintCorrection')
	return finger
	
	
