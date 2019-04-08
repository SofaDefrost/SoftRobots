from splib.numerics import sin, cos, to_radians
from stlib.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
from stlib.physics.collision import CollisionMesh
from splib.objectmodel import SofaPrefab, SofaObject, setData
from stlib.scene import Scene
from stlib.physics.mixedmaterial import Rigidify
from stlib.components import addOrientedBoxRoi
from splib.numerics import vec3
from splib.numerics.quat import Quat


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

    return body


@SofaPrefab
class Tripod(SofaObject):

    def __init__(self, parent, name="Tripod", radius=60, numMotors=3, angleShift=180.0):
        self.node = parent.createChild(name)
        ElasticBody(self.node)

        dist = radius
        numstep = numMotors
        self.actuatedarms = []
        for i in range(0, numstep):
            name = "ActuatedArm"+str(i)
            translation, eulerRotation = self.__getTransform(i, numstep, angleShift, radius, dist)
            self.actuatedarms.append(ActuatedArm(self.node, name=name,
                                                 translation=translation, eulerRotation=eulerRotation))
            self.actuatedarms[i].servomotor.angleLimits = [-2.0225, -0.0255]

        self.__attachToActuatedArms(radius, numMotors, angleShift)

    def __getTransform(self, index, numstep, angleShift, radius, dist):

        fi = float(index)
        fnumstep = float(numstep)
        angle = fi*360/fnumstep
        angle2 = fi*360/fnumstep+angleShift
        eulerRotation = [0, angle, 0]
        translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]

        return translation, eulerRotation

    def addCollision(self, numMotors=3):

        CollisionMesh(self.node.ElasticBody.ElasticMaterialObject, surfaceMeshFileName="data/mesh/tripod_low.stl", name="CollisionModel", translation=[0.0, 30, 0.0], rotation=[90, 0, 0], collisionGroup=1)

        numstep = numMotors
        for i in range(0, numstep):
            CollisionMesh(self.actuatedarms[i].ServoMotor.ServoBody,
                          surfaceMeshFileName="data/mesh/servo_collision.stl",
                          name="TopServoCollision", mappingType='RigidMapping')

    def __attachToActuatedArms(self, radius=60, numMotors=3, angleShift=180.0):

        deformableObject = self.node.ElasticBody.ElasticMaterialObject

        dist = radius
        numstep = numMotors
        groupIndices = []
        frames = []
        for i in range(0, numstep):
            translation, eulerRotation = self.__getTransform(i, numstep, angleShift, radius, dist)
            box = addOrientedBoxRoi(self.node, position=deformableObject.dofs.getData("rest_position"), name="BoxROI"+str(i),
                                    translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                                    eulerRotation=eulerRotation, scale=[45, 15, 30])

            box.drawBoxes = False
            box.init()
            deformableObject.init()
            groupIndices.append([ind[0] for ind in box.indices])
            frames.append(vec3.vadd(translation, [0.0, 25.0, 0.0]) + list(Quat.createFromEuler([0, float(i)*360/float(numstep), 0], inDegree=True)))

        rigidifiedstruct = Rigidify(self.node, deformableObject, groupIndices=groupIndices, frames=frames, name="RigidifiedStruture")

        for i in range(0, numstep):
            rigidifiedstruct.RigidParts.createObject('RestShapeSpringsForceField', name="rssff"+str(i),
                                                     points=i,
                                                     external_rest_shape=self.actuatedarms[i].servoarm.dofs,
                                                     stiffness='1e12', angularStiffness='1e12')

        # Use this to activate some rendering on the rigidified object ######################################
        # setData(rigidifiedstruct.RigidParts.dofs, showObject=True, showObjectScale=10, drawMode=2)
        # setData(rigidifiedstruct.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=1,
        #         drawMode=1, showColor=[1., 1., 0., 1.])
        # setData(rigidifiedstruct.DeformableParts.dofs, showObject=True, showObjectScale=1, drawMode=2)
        #####################################################################################################


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810.0, 0.0])
    rootNode.dt = 0.025
    scene.VisualStyle.displayFlags = "showBehavior"

    scene.createObject("MeshSTLLoader", name="loader", filename="data/mesh/blueprint.stl")
    scene.createObject("OglModel", src="@loader")

    tripod = Tripod(rootNode)
    tripod.createObject("EulerImplicitSolver")
    tripod.createObject("CGLinearSolver")
    tripod.BoxROI0.drawBoxes = True
    tripod.BoxROI1.drawBoxes = True
    tripod.BoxROI2.drawBoxes = True
