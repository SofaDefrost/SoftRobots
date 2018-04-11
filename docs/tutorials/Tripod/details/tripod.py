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

# The components StiffSpringForceField needs a list of springs of the form:
#                [node_i_model1, node_i_model2, stiffness, damping, restLength]
# This list allows to set up each spring between model1 and model2
# The function GenerateSprings generate this list from the following arguments:
#   indices1 : list of indices of model1
#   indices2 : list of indices of model2 (size should be the same of indices1)
#   stiffness : scalar, stiffness of all the springs
#   damping : scalar, damping of all the springs
#   restLength : scalar, length at rest of all the springs
def GenerateSprings(indices1, indices2, stiffness=1e12, damping=5, restLength=0.001):

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
        actuatedArm = get(modelNode, "ActuatedArm"+str(i))
        
        wheel = get(actuatedArm, 'ServoMotor/ServoWheel')
        wheel.createObject('SlidingActuator', template="Rigid", indices="0", direction="0 0 0 1 0 0")

        ## Initialisation needed for the generation of springs
        boxROI = get(actuatedArm, 'Constraint/BoxROI')
        boxROI.init()
        spring = GenerateSprings(range(0,boxROI.nbIndices), get(actuatedArm, 'Constraint/BoxROI.indices').value)

        elasticBody.createObject('StiffSpringForceField',
                                                        name='StiffSpringForceField'+str(i),
                                                        spring=spring,
                                                        object1=get(actuatedArm, 'Constraint/MechanicalObject').getLinkPath(),
                                                        object2="@./")

        elasticBody.createObject('MappedMatrixForceFieldAndMass',
                                                        template="Vec3d,Rigid3d",
                                                        name='MappedMatrixForceFieldAndMass'+str(i),
                                                        object1=get(actuatedArm,'Constraint/MechanicalObject').getLinkPath(),
                                                        object2=get(actuatedArm,'MechanicalObject').getLinkPath(),
                                                        mappedForceField="@./TetrahedronFEMForceField",
                                                        mappedMass="@./UniformMass")


def addSpringConstraint(modelNode, numMotors):
    for i in range(0, numMotors):
            a = get(modelNode, "ActuatedArm"+str(i))

            eb = get(modelNode, "ElasticBody")
            em = eb.getChild("ElasticMaterialObject")
            em.createObject('RestShapeSpringsForceField',
                                     points=get(a, 'Constraint/BoxROI.indices').getLinkPath(),
                                     external_rest_shape=get(a, 'Constraint/MechanicalObject').getLinkPath(),
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

def addInverseSimulationPlan(parentNode, modelNode):
    parentNode.createObject("DefaultAnimationLoop")
    parentNode.createObject("DefaultVisualManagerLoop")
    parentNode.createObject("QPInverseProblemSolver")

    goal = EffectorGoal(parentNode, position=[[0,4,0]], visuScale=0.2)

    part1 = Node(parentNode, "MechanicalPart1")
    part1.createObject("EulerImplicit")
    part1.createObject("SparseLDLSolver")
    part1.addChild( get(modelNode, "ElasticBody") )

    PositionEffector(get(modelNode, "ElasticBody/ElasticMaterialObject"),
                         position=[[0,0,0]],
                         effectorGoal=get(goal, 'MechanicalObject.position').getLinkPath())
    
    part2 = Node(parentNode, "MechanicalPart2")
    part2.createObject("EulerImplicit")
    part2.createObject("CGLinearSolver")
    #part2.addChild( get(modelNode, "ActuatedArm1") )
    #part2.addChild( get(modelNode, "ActuatedArm2") )
    #part2.addChild( get(modelNode, "ActuatedArm3") )


    return parentNode


#Units: cm and kg
def createScene(rootNode):
    import Sofa
    from stlib.scene import MainHeader
    r = MainHeader(rootNode, plugins=["SoftRobots"])
    r.getObject("VisualStyle").displayFlags="showForceFields"
        
    if len(sys.argv) == 2 and sys.argv[1] == "inverse":
        Sofa.msg_info("CreateScene(Tripod)", "Loading the scene with the inverse model")
        m = Node(rootNode, "Model")
        tripod = Tripod(m, invertible=True)

        n = Node(rootNode, "SimulationPlan")
        addInverseSimulationPlan(n, tripod)

    else:
        Sofa.msg_info("CreateScene(Tripod)", "Loading the scene with the direct model")
        m = Node(rootNode, "Model")
        tripod = Tripod(m)

        n = Node(rootNode, "SimulationPlan")
        addDirectSimulationPlan(n, tripod)

	return rootNode
