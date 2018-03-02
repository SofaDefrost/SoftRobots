def PullingCable(attachedTo=None,
    name="cable",
    cableGeometry=[[1.0, 0.0, 0.0],[0.0, 0.0, 0.0]],
    rotation=[0.0,0.0,0.0],
    translation=[0.0,0.0,0.0],
    uniformScale=1.0,
    pullPointLocation=None,
    initialValue=0.0,
    valueType="displacement"):
    """Creates and adds a cable constraint.

    The constraint apply to a parent mesh.

    Args:
        name (str): Name of the created cable.

        cableGeometry: (list vec3f): Location of the degree of freedom of the cable.

        pullPointLocation (vec3f): Position from where the cable is pulled. If not specified
        the point will be considered in the structure.

        valueType (str): either "force" or "displacement". Default is displacement.

        translation (vec3f):   Apply a 3D translation to the object.

        rotation (vec3f):   Apply a 3D rotation to the object in Euler angles.

        uniformScale (vec3f):   Apply an uniform scaling to the object.


    Structure:
        .. sourcecode:: qml

            Node : {
                    name : "cable"
                    MechanicalObject,
                    CableConstraint,
                    BarycentricMapping
            }

    """
    #  This create a new node in the scene. This node is appended to the finger's node.
    cable = attachedTo.createChild(name)

    # This create a MechanicalObject, a componant holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
    cable.createObject('MechanicalObject', position=cableGeometry,
                        rotation=rotation, translation=translation, scale=uniformScale)

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last indice is where the pullPoint is connected.
    if pullPointLocation != None:
        cable.createObject('CableConstraint',
                            indices=range(len(cableGeometry)),
                            pullPoint=pullPointLocation,
                            value=initialValue,
                            valueType=valueType,
                            hasPullPoint=True
                            )
    else:
        cable.createObject('CableConstraint',
                            indices=range(len(cableGeometry)),
                            value=initialValue,
                            valueType=valueType,
                            hasPullPoint=False
                            )

    # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
    # between the cable's DoFs and the parents's ones so that movements of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.createObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)
    
    return cable
    
def createScene(node):
    from stlib.scene import MainHeader
    from stlib.physics.deformable import ElasticMaterialObject

    MainHeader(node, plugins=["SoftRobots"])
    target = ElasticMaterialObject(volumeMeshFileName="mesh/liver.msh",
                                   totalMass=0.5,
                                   attachedTo=node)

    PullingCable(target)
