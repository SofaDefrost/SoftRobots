from splib.numerics import sin, cos, to_radians
from stlib.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
## ajout ci-dessous
from stlib.physics.collision import CollisionMesh


def ElasticBody(parent):
    body = parent.createChild("ElasticBody")

    e = ElasticMaterialObject(body,
                              volumeMeshFileName="data/mesh2/tripod_mid.gidmsh",
                              translation=[0.0,30,0.0], rotation=[90,0,0],
                              youngModulus=600, poissonRatio=0.45, totalMass=0.4)

    visual = body.createChild("Visual")
    visual.createObject("MeshSTLLoader", name="loader", filename="data/mesh2/tripod_mid.stl")
    visual.createObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                        rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.createObject("BarycentricMapping",
                         input=e.dofs.getLinkPath(),
                         output=visual.renderer.getLinkPath())

    # j ai modifie la fonction de collision so I could add a collisionGroup
    # e.addCollisionModel(collisionMesh="data/mesh2/tripod_low.stl",
    #                    translation=[0.0,30,0.0], rotation=[90,0,0])
    #CollisionMesh(e, surfaceMeshFileName="data/mesh2/tripod_low.stl", name="silicone", translation=[0.0, 30, 0.0], rotation=[90, 0, 0], collisionGroup=1)

    return body

def Tripod(parent, name="Tripod", radius=65, numMotors=3, angleShift=180.0):
    tripod = parent.createChild(name)
    body = ElasticBody(tripod)

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

        c = ActuatedArm(tripod, name=name,
                       translation=translation, eulerRotation=eulerRotation,
                       attachingTo=body.ElasticMaterialObject)
    return tripod

## Mon bricolage a partir d ici
def TripodCollision(parent, name="Tripod+Collision", radius=65, numMotors=3, angleShift=180.0):
    tripod = parent.createChild(name)
    body = ElasticBody(tripod)

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

        c = ActuatedArm(tripod, name=name,
                       translation=translation, eulerRotation=eulerRotation,
                       attachingTo=body.ElasticMaterialObject)
        translationServoTops=[dist*sin(to_radians(angle2)), -0.0, dist*cos(to_radians(angle2))+55.0]

        CollisionMesh(c.ServoMotor,
                     surfaceMeshFileName="data/mesh2/servo_collision.stl",
                     name="TopServo"+str(i), collisionGroup=1, mappingType='RigidMapping')

    return tripod
