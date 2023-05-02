import Sofa
from elastic_material_object import ElasticMaterialObject
from stlib3.physics.mixedmaterial import Rigidify
from stlib3.physics.collision import CollisionMesh
from fixing_box import FixingBox
from actuated_arm import ActuatedArm
from stlib3.components import addOrientedBoxRoi

from splib3.constants import Key
import math


class ActuatedFinger(Sofa.Prefab):
    prefabParameters = [
        {"name": "rotation", "type": "Vec3d", "help": "Rotation in base frame", "default": [0.0, 0.0, 0.0]},
        {"name": "translation", "type": "Vec3d", "help": "Translation in base frame",
         "default": [0.0, 0.0, 0.0]}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    # Construct the actuated finger
    def init(self):
        # Load the finger mesh and create an elastic body from it
        self.elasticMaterial = self.elasticBody()
        self.ElasticBody.init()

        # Load a servo motor
        arm = self.addChild(ActuatedArm(name="ActuatedArm", rotation=[90.0, 0, 90.0], translation=[0, 0, 0]))
        arm.ServoMotor.Articulation.dofs.position.value = [[arm.angleIn.value]]  # Initialize the angle
        arm.ServoMotor.minAngle.value = -2.02
        arm.ServoMotor.maxAngle.value = -0.025

        # Define a region of interest to rigidify the nodes of the finger mesh clamped in the servo arm
        box = addOrientedBoxRoi(self,
                                name="boxROIclamped",
                                position=[list(i) for i in self.elasticMaterial.dofs.rest_position.value],
                                translation=[-25e-3, 0.0, 0.0],
                                eulerRotation=[0.0, 0.0, 90.0],
                                scale=[45e-3, 15e-3, 30e-3])
        box.drawBoxes = True
        box.init()

        # Get the indices of the finger mesh in the ROI, and the ROI frame
        indices = [[ind for ind in box.indices.value]]
        frame = [[0, 0, 0, 0, 0, 0, 1]]

        # Rigidify the finger nodes in the ROI. Create a Rigidified object and set up a spring force
        # field to constrain the nodes to stay in the rest shape
        rigidifiedStruct = Rigidify(self, self.elasticMaterial, groupIndices=indices, frames=frame,
                                    name="RigidifiedStructure")

        servoArm = arm.ServoMotor.Articulation.ServoWheel.ServoArm
        servoArm.addChild(rigidifiedStruct.RigidParts)
        servoArm.RigidParts.addObject('RigidMapping', index=0, input=servoArm.dofs.getLinkPath())

        # Add a fixing box to constrain the other part of the finger
        FixingBox(self, self.elasticMaterial, translation=[10.0e-3, 0.0, 14.0e-3], scale=[15e-3, 25e-3, 6e-3])
        self.FixingBox.BoxROI.drawBoxes = True

        # Add collision models, to compute collision between the finger and the object,
        # and the inner surfaces of the left and right walls
        self.addCollision()

    def elasticBody(self):
        # Create a body as a child of the parent (the actuated finger)
        body = self.addChild("ElasticBody")

        # Create an ElasticMaterialObject, which import a mesh, assign it dofs  and mechanical properties
        # All the properties are expressed in SI units. The dimensions in the generated mesh are in meter,
        # the young modulus in Pascal ...
        # The manufacturer of the Filaflex 70A filament indicates a young modulus of 32MPa. The Poisson ratio is fixed
        # at 0.45 like standard elastomers (silicone, rubber)
        # The rotation and translation are adjusted so that the mesh is correctly positioned wrt the servo motor
        e = body.addChild(ElasticMaterialObject(
            volumeMeshFileName="Data/finger.msh",
            topoMesh="tetrahedron",
            scale=[1, 1, 1],
            totalMass=0.015,
            youngModulus=32e6,
            poissonRatio=0.45,
            rotation=[90.0, 0.0, 0.0],
            translation=[-30.0e-3, 9.0e-3, 18.0e-3]))

        # Add now a visual model to the flexible part
        visual = body.addChild("VisualFinger")
        # Load the STL file for the visualization, the rotation and translation must
        # fit the one for the ElasticMaterialObject
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
        CollisionMesh(self.elasticMaterial, name='SelfCollisionMesh1',
                                   surfaceMeshFileName="Data/finger_surface_contact_in1.stl",
                                   rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3])

        CollisionMesh(self.elasticMaterial, name='SelfCollisionMesh2',
                                   surfaceMeshFileName="Data/finger_surface_contact_in2.stl",
                                   rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3])

        CollisionMesh(self.elasticMaterial, name='SelfCollisionMesh3',
                                   surfaceMeshFileName="Data/finger_surface_contact_out.stl",
                                   rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3])


class FingerController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.node = kwargs["node"]
        self.duration = 3.0
        self.time = 0.0
        self.objectDof = kwargs["objectDof"]
        self.actuator = kwargs["actuator"]
        self.forceContact = 0.0
        self.numContact = 0

        # Computation of the contact force applied on the object to grasp
        self.node.getRoot().GenericConstraintSolver.computeConstraintForces.value = True

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.P:
            print("Number of contact points: " + str(self.numContact))
            print("Norm of the contact force: " + str(self.forceContact))

    def onAnimateBeginEvent(self, eventType):

        # Update of the servomotor angular displacement
        # Rotation of pi/6 over self.duration (5s initially)
        angularStep = math.pi / 6
        angleInit = 0
        self.time += self.node.dt.value
        if self.time < self.duration:
            self.actuator.ServoMotor.angleIn = angleInit + angularStep * self.time / self.duration
        else:
            self.actuator.ServoMotor.angleIn = angleInit + angularStep

        # Computation of the contact force applied on the object to grasp
        contactForces = self.node.getRoot().GenericConstraintSolver.constraintForces.value

        # print the number of nodes in contact and the norm of the largest contact force
        self.numContact = 0
        self.forceContact = 0
        for contact in contactForces[0:-1:3]:
            if contact > 0:
                self.numContact += 1
                self.forceContact += contact
        self.forceContact /= self.node.dt.value


def createScene(rootNode):
    from stlib3.scene import Scene

    # Define the main architecture of the scene, with a node Modelling, Setting and Simulation
    # Define also the integration method as Euler implicit and the solver as Conjugate Gradient)
    scene = Scene(rootNode, gravity=[0.0, 0.0, -9.81],
                  iterative=False)
    scene.addMainHeader()

    # Setting the time step
    rootNode.dt = 0.01

    # Define the default view of the scene on SOFA
    scene.addObject('DefaultVisualManagerLoop')
    scene.VisualStyle.displayFlags = ["showInteractionForceFields", "showForceFields",
                                      "showCollisionModels"]
    # Add a grid on the scene with squares 10mm/10mm
    rootNode.addObject("VisualGrid", nbSubdiv=100, size=1)

    # Set up the pipeline for the collision computation
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')
    scene.Settings.mouseButton.stiffness = 10

    # Create one actuated finger
    actuatedFinger = ActuatedFinger()
    scene.Modelling.addChild(actuatedFinger)

    # Add the simulated elements to the Simulation node
    scene.Simulation.addChild(actuatedFinger.RigidifiedStructure.DeformableParts)
    scene.Simulation.addChild(actuatedFinger.ActuatedArm)
    actuatedFinger.ActuatedArm.ServoMotor.Articulation.ServoWheel.ServoArm.dofs.showObject = True
    actuatedFinger.ActuatedArm.ServoMotor.Articulation.ServoWheel.ServoArm.dofs.showObjectScale = 0.01
    actuatedFinger.ActuatedArm.ServoMotor.Articulation.ServoWheel.ServoArm.RigidParts.dofs.showObject = True
    actuatedFinger.ActuatedArm.ServoMotor.Articulation.ServoWheel.ServoArm.RigidParts.dofs.showObjectScale = 0.02