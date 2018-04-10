""" Sofa model for the "tripod' robot

	This model is part of the SoftRobot toolkit available at:
		https://github.com/SofaDefrost/SoftRobots

	List of available parts:
		- Tripod
		- ElasticBody
		- ActuatedArmWithConstraint (add to a s90 ActuatedArm a set of constraints to attach a deformable object )

	Example of use:
		def createScene(rootNode):
			from stlib.scene import MainHeader
			r = MainHeader(rootNode, plugins=["SoftRobots"])
			r.getObject("VisualStyle").displayFlags="showForceFields"

			tripod = Tripod(rootNode)

			return rootNode
"""
from stlib.scene import Node
from stlib.components import OrientedBoxRoi
from stlib.algorithms import get
from stlib.solver import DefaultSolver
from stlib.numerics import *
from math import sin,cos
from parts import ActuatedArm, ServoMotor, ElasticBody

def Tripod(parentNode, numMotors=3, radius=4.6, angleShift=180):
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
		translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]

		c = ActuatedArmWithConstraint(tripod, name=name, position=get(em, "ElasticMaterialObject/MechanicalObject.rest_position").getLinkPath(),
									  translation=translation, eulerRotation=eulerRotation)

	return tripod

def ActuatedArmWithConstraint(parentNode, name="ActuatedArm", position=[], translation=[0,0,0], eulerRotation=[0,0,0]):

	arm = ActuatedArm(parentNode, name=name,
					  translation=translation,
					  eulerRotation=eulerRotation)

	constraint = Node(arm, "Constraint")
	o = OrientedBoxRoi(constraint, position=position,
					 translation=vadd(translation, [0.0,1.0,0.0]),
					 eulerRotation=eulerRotation, scale=[2.5,1,0.9])

	o.drawSize = 5
	constraint.createObject("TransformEngine", input_position="@BoxROI.pointsInROI",
							translation=translation, rotation=eulerRotation, inverse=True )

	constraint.createObject("MechanicalObject", template="Vec3d", position="@TransformEngine.output_position",
							showObject=True, showObjectScale=10.0)

	constraint.createObject('RigidMapping', input="@../ServoMotor/ServoWheel/MechanicalObject", output="@./")

	return arm

def addSimulationPlan(parentNode, modelNode):
	parentNode.createObject("DefaultAnimationLoop")
	parentNode.createObject("DefaultVisualManagerLoop")

	part1 = Node(parentNode, "MechanicalPart1")
	part1.createObject("EulerImplicit")
	part1.createObject("SparseLDLSolver")
	part1.addChild(get(modelNode, "ElasticBody"))

	#get(modelNode, "ElasticBody/ElasticMaterialObject").createObject("LinearSolverConstraintCorrection", solverName="SparseLDLSolver")

	part2 = Node(parentNode, "MechanicalPart2")
	part2.createObject("EulerImplicit")
	part2.createObject("CGLinearSolver")

	for i in range(0,3):
		a = get(modelNode, "ActuatedArm"+str(i))
		part2.addChild(a)

		eb = get(modelNode, "ElasticBody")
		em = eb.getChild("ElasticMaterialObject")
		em.createObject('RestShapeSpringsForceField',
    					 points=get(a, 'Constraint/BoxROI.indices').getLinkPath(),
    					 external_rest_shape=get(a, 'Constraint/MechanicalObject').getLinkPath(),
    					 stiffness='1e12')

	return parentNode

#Units: cm and kg
def createScene(rootNode):
	from stlib.scene import MainHeader
	r = MainHeader(rootNode, plugins=["SoftRobots"])
	r.getObject("VisualStyle").displayFlags="showForceFields"

	m = Node(rootNode, "Model")
	tripod = Tripod(m)

	n = Node(rootNode, "SimulationPlan")
	addSimulationPlan(n, tripod)

	return rootNode
