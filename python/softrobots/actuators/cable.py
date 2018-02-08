def PullingCable(attachedTo=None,
    withName="cable",
    withCableGeometry=[[1.0, 0.0, 0.0],[0.0, 0.0, 0.0]],
    withRotation=[0.0,0.0,0.0],
    withTranslation=[0.0,0.0,0.0],
    withScale=1.0
    withAPullPointLocation=None,
    withInitialValue=0.0,
    withValueAs="displacement"):
    """Creates and adds a cable constraint.

    The constraint apply to a parent mesh.

    Args:
        withName (str): Name of the created cable.

        withCableGeometry: (list vec3f): Location of the degree of freedom of the cable.

        withAPullPointLocation (vec3f): Position from where the cable is pulled. If not specified
        the point will be considered in the structure.

        withValueAs (str): either "force" or "displacement". Default is displacement.

        withTranslation (vec3f):   Apply a 3D translation to the object.

        withRotation (vec3f):   Apply a 3D rotation to the object in Euler angles.

        withScale (vec3f):   Apply an uniform scaling to the object.


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
    cable = attachedTo.createChild(withName)

    # This create a MechanicalObject, a componant holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
    cable.createObject('MechanicalObject', position=withCableGeometry,
                        rotation=withRotation, translation=withTranslation, scale=withScale)

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last indice is where the pullPoint is connected.
    if withAPullPointLocation != None:
        cable.createObject('CableConstraint',
                            indices=range(len(withCableGeometry)),
                            pullPoint=withAPullPointLocation,
                            value=withInitialValue,
                            valueType=withValueAs,
                            hasPullPoint=True
                            )
    else:
        cable.createObject('CableConstraint',
                            indices=range(len(withCableGeometry)),
                            value=withInitialValue,
                            valueType=withValueAs,
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

    MainHeader(node)
    target = ElasticMaterialObject(fromVolumeMesh="mesh/liver.msh",
                                   withTotalMass=0.5,
                                   attachedTo=node)

    PullingCable(target)
