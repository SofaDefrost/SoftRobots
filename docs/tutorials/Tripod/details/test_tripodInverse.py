from stlib.scene import Node
from stlib.components import OrientedBoxRoi
from stlib.algorithms import get
from stlib.solver import DefaultSolver
from stlib.numerics import *
from math import sin,cos
from parts import ActuatedArm, ServoMotor, ElasticBody
from softrobots.inverse.effectors import PositionEffector, EffectorGoal

def Tripod(parentNode, numMotors=3, radius=4.6, angleShift=180):
	bt = BaseTripod(parentNode, numMotors, radius, angleShift)
	addInvertibleSpringConstraint(bt, numMotors)
	return bt

def BaseTripod(parentNode, numMotors=3, radius=4.6, angleShift=180):
	tripod = Node(parentNode, 'Tripod')

	em = ElasticBody(tripod, rotation=[90.0,0.0,0.0],
					 name="ElasticBody")
	eb = em.getChild("ElasticMaterialObject")
	dist = radius
	numstep = numMotors
	for i in range(0,numstep):
		name = "ActuatedArm"+str(i)
		fi = float(i)
		fnumstep = float(numstep)
		angle = fi*360/fnumstep
		angle2 = fi*360/fnumstep+angleShift
		eulerRotation = [0,angle,0]
		# translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]
		translation = [0.,-1.,-5.]

		c = ActuatedArmWithConstraint(tripod, name=name, position=get(em, "ElasticMaterialObject/MechanicalObject.rest_position").getLinkPath(),
									  translation=translation, eulerRotation=eulerRotation)

	return tripod

def ActuatedArmWithConstraint(parentNode, name="ActuatedArm", position=[], translation=[0,0,0], eulerRotation=[0,0,0]):

	armNode = parentNode.createChild(name)

	arm = armNode.createChild("ActuatedArm")
	arm.createObject("MechanicalObject", size=1, position="0 -1 -5 0 0 0 1",
	template='Rigid', showObject=True, showObjectScale=0.5)

	constraint = Node(arm, "Constraint")
	o = OrientedBoxRoi(constraint, position=position,
					 translation=vadd(translation, [0.0,0.75,0.0]),
					 eulerRotation=eulerRotation, scale=[2.5,1.,1])

	get(o, "BoxROI").drawSize = 2
	constraint.createObject("TransformEngine", input_position="@BoxROI.pointsInROI",
							translation=translation, rotation=eulerRotation, inverse=True )

	constraint.createObject("MechanicalObject", template="Vec3d", position="@TransformEngine.output_position",
							showObject=True, showObjectScale=10.0)

	constraint.createObject('RigidMapping', input="@../MechanicalObject", output="@./")

	return arm


# The components StiffSpringForceField needs a list of springs of the form:
#                [node_i_model1, node_i_model2, stiffness, damping, restLength]
# This list allows to set up each spring between model1 and model2
def GenerateSprings(indices1, indices2, stiffness=1e12, damping=5, restLength=0):

	spring = []
	size1 = len(indices1)
	size2 = len(indices2)

	if size1 != size2:
		return spring

	spring = [[0.,0.,0.,0.,0.]]*size1
	for i in range(0,size1):
		spring[i] = [indices1[i],indices2[i],stiffness,damping,restLength]

	return spring


def addInvertibleSpringConstraint(modelNode, numMotors=3):
	## Initialisation needed for the generation of springs
	elasticBody = get(modelNode, "ElasticBody/ElasticMaterialObject")
	get(elasticBody, 'MeshLoader').init()
	get(elasticBody, 'TetrahedronSetTopologyContainer').init()
	get(elasticBody, 'MechanicalObject').init()

	for i in range(0, numMotors):
		actuatedArm = get(modelNode, "ActuatedArm"+str(i)+"/ActuatedArm")

		actuatedArm.createObject('SlidingActuator', template="Rigid", indices="0", direction="0 0 0 1 0 0")

		## Initialisation needed for the generation of springs
		boxROI = get(actuatedArm, 'Constraint/BoxROI')
		boxROI.init()
		spring = GenerateSprings(range(0,boxROI.nbIndices), get(actuatedArm, 'Constraint/BoxROI.indices').value)

		elasticBody.createObject('StiffSpringForceField',
														name='StiffSpringForceField'+str(i),
														spring=spring,
														rayleighStiffness="0.1",
														object1=get(actuatedArm, 'Constraint/MechanicalObject').getLinkPath(),
														object2="@./")


#Units: cm and kg
def createScene(rootNode):
	import Sofa
	import sys
	from stlib.scene import MainHeader
	scene = MainHeader(rootNode, plugins=["SoftRobots"])
	scene.getObject("VisualStyle").displayFlags="showForceFields showInteractionForceFields"
	scene.createObject("FreeMotionAnimationLoop")
	scene.createObject("QPInverseProblemSolver")

	goal = EffectorGoal(scene, position=[[0,4,0]], visuScale=0.2)
	goal.createObject("UncoupledConstraintCorrection")

	tripod = Tripod(scene, numMotors=1)

	get(scene, "Tripod/ElasticBody").createObject("EulerImplicit")
	get(scene, "Tripod/ElasticBody").createObject("SparseLDLSolver")
	get(scene, "Tripod/ElasticBody/ElasticMaterialObject").createObject("LinearSolverConstraintCorrection")

	PositionEffector(get(scene, "Tripod/ElasticBody/ElasticMaterialObject"),
						 position=[[0,0,0]],
						 effectorGoal=get(goal, 'MechanicalObject.position').getLinkPath())

	for i in range(0, 1):
		actuatedArm = get(scene, "Tripod/ActuatedArm"+str(i))
		actuatedArm.createObject("EulerImplicit")
		actuatedArm.createObject("CGLinearSolver", tolerance="0.01", threshold="1e-10")
		actuatedArm = get(scene, "Tripod/ActuatedArm"+str(i)+"/ActuatedArm")
		actuatedArm.createObject('UniformMass', totalmass=0.1)
		actuatedArm.createObject("PartialFixedConstraint", fixedDirections="1 1 1 0 1 1")
		actuatedArm.createObject("UncoupledConstraintCorrection")


		scene.createObject('MappedMatrixForceFieldAndMass',
								template="Vec3d,Vec3d",
								name='MappedMatrixForceFieldAndMass'+str(i),
								rayleighStiffness="0.1",
								object1=get(scene,'Tripod/ElasticBody/ElasticMaterialObject/MechanicalObject').getLinkPath(),
								object2=get(scene,'Tripod/ActuatedArm'+str(i)+'/ActuatedArm/Constraint/MechanicalObject').getLinkPath(),
								mappedForceField="@./Tripod/ElasticBody/ElasticMaterialObject/TetrahedronFEMForceField",
								mappedMass="@./Tripod/ElasticBody/ElasticMaterialObject/UniformMass"
								)

	return rootNode
