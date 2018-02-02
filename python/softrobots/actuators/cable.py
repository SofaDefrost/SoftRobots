def PullingCable(node, name="cable",
    positions=[[1.0, 0.0, 0.0],[0.0, 0.0, 0.0]], pullPoint=[0.0,0.0,0.0]):
    """Creates and adds a cable constraint.

    The constraint apply to a parent mesh.

    Args:
        position (list vec3f): A sequence of points makin a sequential shape

        pullPoint (vec3f): the location of the pulling point.


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
    cable.createObject('MechanicalObject', position=positions)

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last indice is where the pullPoint is connected.
    cable.createObject('CableConstraint',
                        indices=range(len(position)),
                        pullPoint=pullPoint)
                        
    # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
    # between the cable's DoFs and the parents's ones so that movements of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.createObject('BarycentricMapping', name="mapping")
    
    return cable
    
def createScene(node):
    node.createObject('MechanicalObject')
    Cable(node)
