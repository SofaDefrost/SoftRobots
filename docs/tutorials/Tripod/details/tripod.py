from splib.numerics import sin, cos, to_radians, vec3, Quat
from stlib.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
from stlib.physics.collision import CollisionMesh
from stlib.physics.mixedmaterial import Rigidify
from splib.objectmodel import SofaPrefab, SofaObject
from stlib.scene import Scene

def ElasticBody(parent):
    body = parent.createChild("ElasticBody")

    e = ElasticMaterialObject(body,
                              volumeMeshFileName="data/mesh/tripod_mid.gidmsh",
                              translation=[0.0, 30, 0.0], rotation=[90, 0, 0],
                              youngModulus=800, poissonRatio=0.45, totalMass=0.032)

    visual = body.createChild("Visual")
    visual.createObject("MeshSTLLoader", name="loader", filename="data/mesh/tripod_mid.stl")
    visual.createObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                        rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.createObject("BarycentricMapping",
                        input=e.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    CollisionMesh(e, surfaceMeshFileName="data/mesh/tripod_low.stl", name="CollisionMesh", translation=[0.0, 30, 0.0], rotation=[90, 0, 0], collisionGroup=1)

    body.init()    
    return body



# Let's define a Tripod prefab now, that we can later call in the createScene function
@SofaPrefab
class Tripod(SofaObject):
    def __init__(self, parent, name="Tripod", radius=60, numMotors=3, angleShift=180.0):
        self.node = parent.createChild(name)

        # It is using the ElasticBody that was made in the previous step, and that has also been included in the tripod.py script.
        body = ElasticBody(self.node)
        
        # The actuated arms are positionned around the silicone piece using a loop structure
        dist = radius
        numstep = numMotors
        self.actuatedarms = []
        groupIndices = []
        frames = []
        handles = []
        for i in range(0, numstep):
            name = "ActuatedArm"+str(i)
            fi = float(i)
            fnumstep = float(numstep)
            angle = fi*360/fnumstep
            angle2 = fi*360/fnumstep+angleShift
            eulerRotation = [0, angle, 0]
            translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]


            arm = ActuatedArm(self.node, name=name, translation=translation, eulerRotation=eulerRotation)
            handle = arm.addHandle(translation=translation, eulerRotation=eulerRotation)
            handles.append(handle)
            
            handle.boxroi.position = body.ElasticMaterialObject.dofs.getData("position").getLinkPath()  
            handle.boxroi.drawPoints = True
            handle.boxroi.drawBoxes = True
            handle.boxroi.drawSize = 2
            
            groupIndices.append([ind[0] for ind in handle.boxroi.indices])
            frames.append(vec3.vadd(translation, [0.0, 25.0, 0.0]) + 
                          list(Quat.createFromEuler([0, float(i)*360/float(numstep), 0], inDegree=True)))

            self.actuatedarms.append(arm)
        
        rigidifiedstruct = Rigidify(self.node, body.ElasticMaterialObject, groupIndices=groupIndices, frames=frames, name="RigidifiedStructure")

        for i,handle in enumerate(handles):
                rigidifiedstruct.RigidParts.createObject('RestShapeSpringsForceField', 
                                                         name="rsforcefield"+str(i),
                                                         points=i,
                                                         external_rest_shape=handle.slavedofs,
                                                         stiffness='1e12', angularStiffness='1e12')

    def addCollision(self):
        for arm in self.actuatedarms:
            CollisionMesh(arm.ServoMotor.BaseFrame,
                          surfaceMeshFileName="data/mesh/servo_collision.stl",
                          name="TopServoCollision", mappingType='RigidMapping')



@SofaPrefab
class TripodOld(SofaObject):

    def __init__(self, parent, name="Tripod", radius=60, numMotors=3, angleShift=180.0):
        self.node = parent.createChild(name)
        body = ElasticBody(self.node)

        dist = radius
        numstep = numMotors
        self.actuatedarms = []
        for i in range(0, numstep):
            name = "ActuatedArm"+str(i)
            fi = float(i)
            fnumstep = float(numstep)
            angle = fi*360/fnumstep
            angle2 = fi*360/fnumstep+angleShift
            eulerRotation = [0, angle, 0]
            translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]

            self.actuatedarms.append(ActuatedArm(self.node, name=name,
                                                 translation=translation, eulerRotation=eulerRotation,
                                                 attachingTo=body.ElasticMaterialObject))
            self.actuatedarms[i].servomotor.angleLimits = [-2.0225, -0.0255]

        #sself.node.init()

    def addCollision(self, numMotors=3):
        numstep = numMotors
        for i in range(0, numstep):
            CollisionMesh(self.actuatedarms[i].ServoMotor.ServoBody,
                          surfaceMeshFileName="data/mesh/servo_collision.stl",
                          name="TopServoCollision", mappingType='RigidMapping')


def createScene(rootNode):
    from splib.objectmodel import setData, setTreeData
    
    scene = Scene(rootNode, gravity=[0.0, -9810.0, 0.0])
    setData(scene, dt=0.025)
    setData(scene.VisualStyle, displayFlags = "showBehavior")

    simulation = scene.createChild("Simulation")
    simulation.createObject("EulerImplicitSolver")
    simulation.createObject("CGLinearSolver", iterations=120, tolerance=1e-20, threshold=1e-20)

    tripod = Tripod(simulation)
    setTreeData(tripod, "boxroi", drawBoxes=True)

