import Sofa
from elastic_material_object import ElasticMaterialObject
from stlib3.physics.mixedmaterial import Rigidify
from stlib3.physics.collision import CollisionMesh
from fixing_box import FixingBox
from actuated_arm import ActuatedArm
from stlib3.components import addOrientedBoxRoi

from splib3.numerics.quat import Quat
from splib3.numerics import vec3

from numpy.linalg import norm
from splib3.constants import Key
import math

class ActuatedFinger(Sofa.Prefab):
	properties = [{"name":"name",			"type":"string", 		"help":"Node name", 				"default":"ActuatedFinger"},
				{"name":"rotation",			"type":"Vec3d", 		"help":"Rotation in base frame", 	"default":[0.0,0.0,0.0]},
				{"name":"translation",		"type":"Vec3d", 		"help":"Translation in base frame", "default":[0.0,0.0,0.0]}]

	def __init__(self,*args,**kwargs):
		Sofa.Prefab.__init__(self,*args,**kwargs)

	# Construct the actuated finger
	def init(self):
		#Load the finger mesh and create an elastic body from it
		self.elasticMaterial = self.elasticBody()
		self.ElasticBody.init()

		#Load a servo motor
		arm = self.addChild(ActuatedArm(name = "ActuatedArm",rotation=[90.0,0,90.0],translation=[0,0,0]))
		arm.angleIn.value = -1.57
		arm.ServoMotor.minAngle.value =-2.02
		arm.ServoMotor.maxAngle.value = -0.025

		#Define a region of interest to rigidify the nodes of the finger mesh clampled in the servo arm
		box = addOrientedBoxRoi(self,
							name = "boxROIclamped",
							position = [list(i) for i in self.elasticMaterial.dofs.rest_position.value],
							translation = [-25e-3,0.0,0.0],
							eulerRotation = [0.0,0.0,90.0],
							scale = [45e-3,15e-3,30e-3])
		box.drawBoxes = True
		box.init()

		# Get the indices of the finger mesh in the ROI, and the ROI frame
		index = []
		frame = []
		index.append([ind for ind in box.indices.value])
		frame.append([-25e-3,0.0,0.0] + list(Quat.createFromEuler([0.0,0.0,90.0],inDegree=True)))

		# Rigidify the finger nodes in the ROI. Create a Rigidify object and setup a spring force field to constrain the nodes to stay in the rest shape
		rigidifiedStruct = Rigidify(self,self.elasticMaterial,groupIndices=index,frames=frame,name="RigidifiedStructure")
		rigidifiedStruct.RigidParts.addObject("RestShapeSpringsForceField",name="rssff",
												points=0,
												external_rest_shape=arm.servoarm.dofs.getLinkPath(),
												stiffness = 1e12,
												angularStiffness = 1e12)

		# Add a fixing box to constrain the other part of the finger
		FixingBox(self,self.elasticMaterial,translation=[10.0e-3,0.0,14.0e-3],scale=[15e-3,25e-3,6e-3])
		self.FixingBox.BoxROI.drawBoxes=True

		# Add collision models, to compute collision between the finger and the object,
		# and the inner surfaces of the left and right walls
		self.addCollision()

	def elasticBody(self):
		# Create a body as a child of the parent (the actuated finger)
		body = self.addChild("ElasticBody")

		# Create an ElasticMaterialObject, which import a mesh, assign it dofs  and mechanical properties
		# All the properties are expressed in SI units. The dimensions in the generated mesh are in meter, the young modulus in Pascal ...
		# The rotation and translation are adusted so that the mesh is correctly positioned wrt the servo motor
		e = body.addChild(ElasticMaterialObject(
			volumeMeshFileName="Data/finger.msh",
			topoMesh="tetrahedron",
			scale=[1, 1, 1],
			totalMass=0.010,
			youngModulus=12e6,
			poissonRatio=0.45,
			rotation=[90.0, 0.0, 0.0],
			translation=[-30.0e-3, 9.0e-3, 18.0e-3]))

		# Add now a visual model to the flexible part
		visual = body.addChild("VisualFinger")
		# Load the STL file for the vizualization, the rotation and translation must fit the one for the ElasticMaterialObject
		visual.addObject("MeshSTLLoader", name="visualLoader", filename="Data/finger.stl", rotation=[90.0, 0.0, 0.0],
						 translation=[-30.0e-3, 9.0e-3, 18.0e-3])
		visual.addObject("OglModel", name="renderer", src="@visualLoader", color=[1.0, 1.0, 1.0, 0.5])

		# Link the dofs of the 3D mesh and the visual model
		visual.addObject("BarycentricMapping", input=e.dofs.getLinkPath(), output=visual.renderer.getLinkPath())

		return e

	def addCollision(self):
		# Add a collision model
		# collision = CollisionMesh(self.elasticMaterial, name='CollisionMesh', surfaceMeshFileName="Data/finger.stl",
		# 						  rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3],
		# 						  collisionGroup=[1, 2, 3])
		collision2 = CollisionMesh(self.elasticMaterial, name='SelfCollisionMesh1',
								   surfaceMeshFileName="Data/finger_surface_contact_in1.stl",
								   rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3],
								   collisionGroup=[1])
		Ks = 3.0
		collision2.PointCollisionModel.contactStiffness = Ks
		collision2.LineCollisionModel.contactStiffness = Ks
		collision2.TriangleCollisionModel.contactStiffness = Ks

		collision3 = CollisionMesh(self.elasticMaterial, name='SelfCollisionMesh2',
								   surfaceMeshFileName="Data/finger_surface_contact_in2.stl",
								   rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3],
								   collisionGroup=[2])
		collision3.PointCollisionModel.contactStiffness = Ks
		collision3.LineCollisionModel.contactStiffness = Ks
		collision3.TriangleCollisionModel.contactStiffness = Ks

		collision4 = CollisionMesh(self.elasticMaterial, name='SelfCollisionMesh3',
								   surfaceMeshFileName="Data/finger_surface_contact_out.stl",
								   rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3],
								   collisionGroup=[3])
		collision4.PointCollisionModel.contactStiffness = Ks
		collision4.LineCollisionModel.contactStiffness = Ks
		collision4.TriangleCollisionModel.contactStiffness = Ks


class FingerController(Sofa.Core.Controller):

	def __init__(self, *args, **kwargs):
		# These are needed (and the normal way to override from a python class)
		Sofa.Core.Controller.__init__(self, *args, **kwargs)

		self.node = kwargs["node"]
		self.duration = 5.0
		self.time= 0.0
		self.objectDof = kwargs["objectDof"]
		self.actuator = kwargs["actuator"]
		self.forceContact = 0.0
		self.numContact = 0
		self.prevForce = []


	def onKeypressedEvent(self,event):
		key = event['key']
		if key==Key.P:
			print("Number of contact points: " + str(self.numContact))
			print("Norm of the contact force: " + str(self.forceContact))

	def onAnimateBeginEvent(self,eventType):

		# Update of the servomotor angular displacement
		# Rotation of pi/6 over self.duration (5s initially)
		angularStep = math.pi / 6
		angleInit = -math.pi / 2
		self.time += self.node.dt.value
		if self.time<self.duration:
			self.actuator.ServoMotor.angleIn = angleInit + angularStep*self.time/self.duration
		else:
			self.actuator.ServoMotor.angleIn = angleInit + angularStep

		# Computation of the contact force applied on the object to grasp
		contactForces = self.objectDof.force
		size = len(contactForces)
		normF = [0.0]

		# Initialize the vector of contact forces at the previous time step if it has not been already
		if len(self.prevForce) == 0:
			for k in range(0, size):
				self.prevForce.append([0.0, 0.0, 0.0])

		# The "force" field of the collision model mechanical object corresponds to the integral
		# over time of the contact force. At each time step, we compute the time derivative of "force"
		# to obtain the contact force
		for k in range(0, size):
			if norm(contactForces[k]) != 0:
				dForce = vec3.sdiv(vec3.vsub(contactForces[k], self.prevForce[k]), self.node.dt.value)
				normF.append(norm(dForce))
			vec = contactForces[k]
			self.prevForce[k] = [vec[0],vec[1],vec[2]]

		# print the number of nodes in contact and the norm of the largest contact force
		self.numContact = len(normF) - 1
		self.forceContact = max(normF)



