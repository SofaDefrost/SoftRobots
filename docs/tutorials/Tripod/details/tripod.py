from splib.numerics import sin, cos, to_radians
from stlib.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
from stlib.physics.collision import CollisionMesh
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

    CollisionMesh(e, surfaceMeshFileName="data/mesh/tripod_low.stl", name="silicone", translation=[0.0, 30, 0.0], rotation=[90, 0, 0], collisionGroup=1)

    return body


@SofaPrefab
class Tripod(SofaObject):

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

    def addCollision(self, numMotors=3):

        numstep = numMotors
        for i in range(0, numstep):
            CollisionMesh(self.actuatedarms[i].ServoMotor.ServoBody,
                          surfaceMeshFileName="data/mesh/servo_collision.stl",
                          name="TopServoCollision", mappingType='RigidMapping')


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810.0, 0.0])
    rootNode.dt = 0.025
    scene.VisualStyle.displayFlags = "showBehavior"

    scene.createObject("MeshSTLLoader", name="loader", filename="data/mesh/blueprint.stl")
    scene.createObject("OglModel", src="@loader")

    tripod = Tripod(rootNode)
    for arm in tripod.actuatedarms:
        arm.Constraint.BoxROI.drawBoxes = True
