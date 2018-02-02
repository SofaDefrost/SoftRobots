
def Cable(node, name="cable", position=[[1.0, 0.0, 0.0],[0.0, 0.0, 0.0]], pullPoint=[0.0,0.0,0.0]):
    #  This create a new node in the scene. This node is appended to the finger's node.
    cable = node.createChild(name)

    # This create a MechanicalObject, a componant holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    # the points where the cable is passing by.
    cable.createObject('MechanicalObject', name="mstate", position=position)

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last indice is where the pullPoint is connected.
    cable.createObject('CableConstraint', name="constraint",
                        indices=range(len(position)),
                        pullPoint=pullPoint)
                        
    # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
    # between the cable's DoFs and the finger's ones so that movements of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.createObject('BarycentricMapping', name="mapping")
    
    return cable
    
def createScene(node):
    node.createObject('MechanicalObject')
    Cable(node)
