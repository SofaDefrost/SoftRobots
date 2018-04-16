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
from splib.numerics import *
from splib.scenegraph import get

from stlib.scene import Node
from stlib.components import addOrientedBoxRoi
from stlib.solver import DefaultSolver
from math import sin,cos
from parts import ActuatedArm, ServoMotor, ElasticBody
from softrobots.inverse.effectors import PositionEffector, EffectorGoal

def Tripod(parentNode, numMotors=3, radius=4.6, angleShift=180, invertible=False):
    bt = BaseTripod(parentNode, numMotors, radius, angleShift)
    if invertible:
        addInvertibleSpringConstraint(bt, numMotors)
    else:
        addSpringConstraint(bt, numMotors)
    return bt

def BaseTripod(parentNode, numMotors=3, radius=4.6, angleShift=180):
	tripod = Node(parentNode, 'Tripod')

	elasticbody = ElasticBody(tripod, eulerRotation=[90.0,0.0,0.0], name="ElasticBody")
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

		c = ActuatedArmWithConstraint(tripod, name=name, 
                                      position=elasticbody.dofs.findData("rest_position"),
									  translation=translation, eulerRotation=eulerRotation)

	return tripod

def ActuatedArmWithConstraint(parentNode, name="ActuatedArm", position=[], translation=[0,0,0], eulerRotation=[0,0,0]):
    arm = ActuatedArm(parentNode, name=name,
					  translation=translation,
					  eulerRotation=eulerRotation)

    constraint = Node(arm.node, "Constraint")
    o = addOrientedBoxRoi(constraint, position=position,
					 translation=vadd(translation, [0.0,1.0,0.0]),
					 eulerRotation=eulerRotation, scale=[2.5,1,1])

    o.drawSize = 5
    constraint.createObject("TransformEngine", input_position="@BoxROI.pointsInROI",
							translation=translation, rotation=eulerRotation, inverse=True )

    constraint.createObject("MechanicalObject", name="dofs", 
                            template="Vec3d", position="@TransformEngine.output_position", 	           
    						showObject=True, showObjectScale=10.0)

    constraint.createObject('RigidMapping', name="mapping", input=arm.servomotor.servowheel.dofs, output="@./")

    return arm

def addSpringConstraint(modelNode, numMotors):
    for i in range(0, numMotors):
            a = get(modelNode, "ActuatedArm"+str(i))

            eb = get(modelNode, "ElasticBody")
            eb.createObject('RestShapeSpringsForceField',
                                     points=get(a, 'Constraint/BoxROI.indices').getLinkPath(),
                                     external_rest_shape=get(a, 'Constraint/dofs').getLinkPath(),
                                     stiffness='1e12')


def addDirectSimulationPlan(parentNode, modelNode):
    parentNode.createObject("DefaultAnimationLoop")
    parentNode.createObject("DefaultVisualManagerLoop")

    part1 = Node(parentNode, "MechanicalPart1")
    part1.createObject("EulerImplicit")
    part1.createObject("SparseLDLSolver")
    part1.addChild(get(modelNode, "ElasticBody"))

    part2 = Node(parentNode, "MechanicalPart2")
    part2.createObject("EulerImplicit")
    part2.createObject("CGLinearSolver")
    for i in range(0, 3):
        part2.addChild(get(modelNode, "ActuatedArm"+str(i)))

    return parentNode

#Units: cm and kg
def createScene(rootNode):
    import Sofa
    from stlib.scene import MainHeader
    r = MainHeader(rootNode, plugins=["SoftRobots"])
    r.getObject("VisualStyle").displayFlags="showForceFields showInteractionForceFields"

    if len(sys.argv) == 2 and sys.argv[1] == "inverse":
        Sofa.msg_info("CreateScene(Tripod)", "Loading the scene with the inverse model")
        m = Node(rootNode, "Model")
        tripod = Tripod(m, invertible=True, numMotors=1)

        n = Node(rootNode, "SimulationPlan")
        #addInverseSimulationPlan(n, tripod)

    else:
        Sofa.msg_info("CreateScene(Tripod)", "Loading the scene with the direct model")
        m = Node(rootNode, "Model")
        tripod = Tripod(m)

        n = Node(rootNode, "SimulationPlan")
        addDirectSimulationPlan(n, tripod)

    return rootNode
