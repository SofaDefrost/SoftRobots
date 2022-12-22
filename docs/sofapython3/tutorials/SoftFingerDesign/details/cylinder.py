import Sofa
from stlib3.visuals import VisualModel


def Cylinder(name="Cylinder",
             surfaceMeshFileName=None,
             translation=[0., 0., 0.],
             rotation=[0., 0., 0.],
             MOscale=1.,
             length=1.,
             uniformScale=1.,
             totalMass=1.,
             volume=1.,
             inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
             color=[1., 1., 1., 1.],
             collisionGroup='',
             isAStaticObject=False, parent=None):
    """Creates and adds rigid body from a surface mesh.
    Args:
        surfaceMeshFileName (str):  The path or filename pointing to surface mesh file.
        totalMass (float):   The mass is distributed according to the geometry of the object.
        color (vec4f):  The default color used for the rendering of the object.
        translation (vec3f):   Apply a 3D translation to the object.
        rotation (vec3f):   Apply a 3D rotation to the object in Euler angles.
        uniformScale (vec3f):   Apply a uniform scaling to the object.
        isAStaticObject (bool): The object does not move in the scene (e.g. floor, wall) but react to collision.
    Structure:
            .. sourcecode:: qml
                Node : {
                    name : "rigidobject"
                    MechanicalObject,
                    UniformMass,
                    UncoupledConstraintCorrection,
                    *EulerImplicit,
                    *SparseLDLSolver,
                    Node : {
                        name : "collision",
                        Mesh,
                        MechanicalObject,
                        Triangle,
                        Line,
                        Point,
                        RigidMapping
                    }
                    Node : {
                       name : "visual"
                       OglModel,
                       RigidMapping
                    }
                }
    """

    # mechanics
    object = Sofa.Core.Node(name)

    if parent is not None:
        parent.addChild(object)

    object.addObject('MechanicalObject',
                     name="mstate", template="Rigid3",
                     translation2=translation, rotation2=rotation, showObjectScale=MOscale)

    object.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    if not isAStaticObject:
        object.addObject('EulerImplicitSolver')
        object.addObject('CGLinearSolver')

    def addCollisionModel(inputMesh=surfaceMeshFileName):
        objectCollis = object.addChild('collision')
        objectCollis.addObject('MeshSTLLoader', name="loader",
                               filename=inputMesh, triangulate=True,
                               scale=uniformScale)

        objectCollis.addObject('MeshTopology', src="@loader")
        objectCollis.addObject('MechanicalObject')

        if isAStaticObject:
            objectCollis.addObject('TriangleCollisionModel', moving=False, simulated=False, group=collisionGroup)
            objectCollis.addObject('LineCollisionModel', moving=False, simulated=False, group=collisionGroup)
            objectCollis.addObject('PointCollisionModel', moving=False, simulated=False, group=collisionGroup)
        else:
            objectCollis.addObject('TriangleCollisionModel', group=collisionGroup)
            objectCollis.addObject('LineCollisionModel', group=collisionGroup)
            objectCollis.addObject('PointCollisionModel', group=collisionGroup)
        objectCollis.addObject('RigidMapping')

    object.addCollisionModel = addCollisionModel

    # visualization
    def addVisualModel(inputMesh=surfaceMeshFileName):
        visual = VisualModel(name="visual", visualMeshPath=inputMesh, color=color, scale=[uniformScale] * 3)
        object.addChild(visual)
        visual.addObject('RigidMapping')

    object.addVisualModel = addVisualModel

    if surfaceMeshFileName is not None:
        object.addCollisionModel()
        object.addVisualModel()

    return object
