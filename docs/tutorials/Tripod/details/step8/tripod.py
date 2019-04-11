from splib.numerics import sin, cos, to_radians
from stlib.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
from stlib.physics.collision import CollisionMesh


def ElasticBody(parent):
    body = parent.createChild("ElasticBody")

    e = ElasticMaterialObject(body,
                              volumeMeshFileName="../data/mesh/tripod_mid.gidmsh",
                              translation=[0.0,30,0.0], rotation=[90,0,0],
                              youngModulus=600, poissonRatio=0.45, totalMass=0.4)

    visual = body.createChild("Visual")
    visual.createObject("MeshSTLLoader", name="loader", filename="../data/mesh/tripod_mid.stl")
    visual.createObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                        rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.createObject("BarycentricMapping",
                         input=e.dofs.getLinkPath(),
                         output=visual.renderer.getLinkPath())
    return body
