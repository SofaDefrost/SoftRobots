def PullingCable(node, name="cable",
    position, value=0, valueType="displacement", pullPoint):
    """Creates and adds a cable constraint.

    The constraint apply to a parent mesh.

    Args:
        position (vec<vec3>): cable path into the volume structure
        
        value (real): value to apply, default is 0.
        
        valueType (string): type of the parameter value (displacement or force), default is displacement.
        
        pullPoint (vec3): position from where the cable is pulled (if not specified the pullPoint will be considered on the structure) 


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
    cable = node.createChild(name)

    # This create a MechanicalObject, a componant holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
    cable.createObject('MechanicalObject', position=position)

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last indice is where the pullPoint is connected.
    if len(pullPoint) != 0:
        cable.createObject('CableConstraint',
                            indices=range(len(position)),
                            hasPullPoint=true,
                            pullPoint=pullPoint)
    else:
        cable.createObject('CableConstraint',
                            indices=range(len(position)),
                            hasPullPoint=false)
                        
    # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
    # between the cable's DoFs and the parents's ones so that movements of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.createObject('BarycentricMapping', name="mapping", mapForces="false", mapMasses="false")
    
    return cable
    
def createScene(node):
    node.createObject('MechanicalObject')
    Cable(node)
