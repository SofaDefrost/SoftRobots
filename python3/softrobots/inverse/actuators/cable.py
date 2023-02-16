def PullingCable(attachedTo=None,
    name="Cable",
    cableGeometry=[[1.0, 0.0, 0.0],[0.0, 0.0, 0.0]],
    pullPointLocation=None,
    minForce=None,
    maxForce=None,
    minDisplacement=None,
    maxDisplacement=None,
    maxDispVariation=None,
    translation=[0.0,0.0,0.0],
    rotation=[0.0,0.0,0.0],
    uniformScale=1.0):
    """Adds a cable actuators. Should be used in the context of the resolution of an inverse problem: find the actuation that leads to a desired deformation.
    See documentation at: https://project.inria.fr/softrobot/documentation/constraint/cable-actuator/

    The constraint is applied to a parent mesh.

    Args:
        name (str): Name of the added cable.

        cableGeometry: (list vec3): Location of the degree of freedom of the cable.

        pullPointLocation (vec3): Position from where the cable is pulled. If not specified
        the point will be considered in the structure.

        minForce:

        maxForce:

        minDisplacement:

        maxDisplacement:

        maxDispVariation:

        translation (vec3):   Apply a 3D translation to the object.

        rotation (vec3):   Apply a 3D rotation to the object in Euler angles.

        uniformScale (vec3):   Apply an uniform scaling to the object.


    Structure:
        .. sourcecode:: qml

            Node : {
                    name : "Cable"
                    MechanicalObject,
                    CableActuator,
                    BarycentricMapping
            }

    """
    #  This add a new node in the scene. This node is appended to the model's node.
    cable = attachedTo.addChild(name)

    # This add a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
    cable.addObject('MechanicalObject', position=cableGeometry,
                        rotation=rotation, translation=translation, scale=uniformScale)

    # Add a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last indice is where the pullPoint is connected.
    if pullPointLocation != None:
        cable.addObject('CableActuator',
                            indices=range(len(cableGeometry)),
                            pullPoint=pullPointLocation,
                            hasPullPoint=True
                            )
    else:
        cable.addObject('CableActuator',
                            indices=range(len(cableGeometry)),
                            hasPullPoint=False
                            )

    if minForce != None : cable.minForce = minForce
    if maxForce != None : cable.maxForce = maxForce
    if minDisplacement != None : cable.minDisplacement = minDisplacement
    if maxDisplacement != None : cable.maxDisplacement = maxDisplacement
    if maxDispVariation != None : cable.maxDispVariation = maxDispVariation

    # This add a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    # between the cable's DoFs and the parents's ones so that movements of the cable's DoFs will be mapped
    # to the model and vice-versa;
    cable.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)

    return cable
