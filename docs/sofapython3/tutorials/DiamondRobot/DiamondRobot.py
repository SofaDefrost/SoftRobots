import Sofa

import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'
meshRobot = path + 'diamond.vtk'


def createScene(rootNode):
    # Root node
    rootNode.findData('dt').value = 0.1
    rootNode.findData('gravity').value = [0, 0, -9810]
    rootNode.addObject('VisualStyle',
                       displayFlags='showCollision showVisualModels showForceFields showInteractionForceFields hideCollisionModels hideBoundingCollisionModels hideWireframe')

    # Required plugin
    rootNode.addObject('RequiredPlugin', name='ExternalPlugins', pluginName='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaPlugins',
                       pluginName=['SofaBoundaryCondition', 'SofaImplicitOdeSolver', 'SofaPreconditioner',
                                   'SofaSimpleFem', 'SofaSparseSolver', 'SofaLoader', 'SofaEngine', 'SofaConstraint'])

    # Constraint solver, here we use a Gauss Seidel algorithm
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=500, tolerance=1e-8)

    # Robot
    robot = rootNode.addChild('Robot')
    # The solvers
    robot.addObject('EulerImplicitSolver')
    robot.addObject('ShewchukPCGLinearSolver', iterations=1, name="linearsolver", tolerance=1e-5,
                    preconditioners="preconditioner", use_precond=True)
    robot.addObject('SparseLDLSolver', name="preconditioner", template="CompressedRowSparseMatrixMat3x3d")
    # Load the volume mesh
    robot.addObject('MeshVTKLoader', name="loader", filename=meshRobot)
    robot.addObject('MeshTopology', src="@loader")
    robot.addObject('MechanicalObject', name="tetras", template="Vec3", showIndices=False, showIndicesScale=4e-5, rx=90,
                    dz=35)
    # Set the mechanical parameters
    robot.addObject('UniformMass', totalMass=0.5)
    robot.addObject('TetrahedronFEMForceField', youngModulus=180, poissonRatio=0.45)
    # Fix a part of the model
    robot.addObject('BoxROI', name="boxROI", box=[-15, -15, -40, 15, 15, 10], drawBoxes=True)
    robot.addObject('FixedConstraint', indices="@boxROI.indices")
    robot.addObject('LinearSolverConstraintCorrection', solverName="preconditioner")

    # Actuators
    actuators = robot.addChild('Actuators')
    # Points on the model where the cables are attached
    actuators.addObject('MechanicalObject', template="Vec3",
                        position=[[0, 0, 125], [0, 97, 45], [-97, 0, 45], [0, -97, 45], [97, 0, 45], [0, 0, 115]])
    # Cables
    actuators.addObject('CableConstraint', template="Vec3", name="north",
                        indices=1,  # indice in the MechanicalObject of the corresponding attach point
                        pullPoint=[0, 10, 30],  # point from where the cable is being pulled
                        valueType='displacement',
                        # choose if you want to control the displacement or the force of the cable
                        value=20)  # the displacement or force to apply
    actuators.addObject('CableConstraint', template="Vec3", name="west", indices=2, pullPoint=[-10, 0, 30])
    actuators.addObject('CableConstraint', template="Vec3", name="south", indices=3, pullPoint=[0, -10, 30])
    actuators.addObject('CableConstraint', template="Vec3", name="east", indices=4, pullPoint=[10, 0, 30])
    # This component is used to map the attach points onto the FEM mesh
    actuators.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    return rootNode
