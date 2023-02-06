def PositionEffector(attachedTo=None,
                    name="effector",
                    position=None,
                    effectorGoal=None,
                    template="Vec3",
                    directions=None,
                    useDirections=None,
                    translation=[0.0,0.0,0.0],
                    rotation=[0.0,0.0,0.0],
                    uniformScale=1.0):

    """Adds a position effector. Should be used in the context of the resolution of an inverse problem: find the actuation that leads to a desired position of the effector.
    See documentation at: https://project.inria.fr/softrobot/documentation/constraint/position-effector/

    The constraint apply to a parent mesh.

    Args:
        name (str): Name of the added effector.

        position: Location of the degree of freedom of the effector.

        effectorGoal: Location of the effector target

        template:

        translation (vec3):   Apply a 3D translation to the object.

        rotation (vec3):   Apply a 3D rotation to the object in Euler angles.

        uniformScale (vec3):   Apply an uniform scaling to the object.


    Structure:
        .. sourcecode:: qml

            Node : {
                    name : "effector"
                    MechanicalObject,
                    PositionEffector,
                    BarycentricMapping or RigidMapping
            }

    """
    #  This add a new node in the scene. This node is appended to the model's node.
    effector = attachedTo.addChild(name)

    # This add a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a effector it is a set of positions specifying
    # ghe location of the effector
    effector.addObject('MechanicalObject', template=template, position=position,
                        rotation=rotation, translation=translation, scale=uniformScale)

    # Add a PositionEffector object with a name.
    # the indices are referring to the MechanicalObject's positions.
    positionEffector = effector.addObject('PositionEffector', template=template,
                                            indices=range(len(position)),
                                            effectorGoal=effectorGoal
                                            )
    if directions != None : positionEffector.directions = directions
    if useDirections != None : positionEffector.useDirections = useDirections

    # This add a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    # between the effector's DoFs and the parents's ones so that movements of the effector's DoFs will be mapped
    # to the model and vice-versa;
    if template == "Rigid3d" or template == "Rigid3f":
        effector.addObject('RigidMapping', name="Mapping", mapForces=False, mapMasses=False)
    else:
        effector.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)

    return effector
