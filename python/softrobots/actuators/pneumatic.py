def getOrCreateTheTemplateNode(attachedAsAChildOf=None, attachedTo=None, withName=None):
    if attachedTo != None:
        if withName != None:
            Sofa.msg_error(attachedTo, "The withName parameter cannot be used when a template is attachedTo to an existing node")
            return attachedTo
        if attachedAsAChildOf != None:
            Sofa.msg_error(attachedTo, "Both attachedTo and attachedAsAChildOf are set is not allowed.")
            return attachedTo
        return attachedTo
    return attachedAsAChildOf.createChild(withName)

def PneumaticCavity(fromSurfaceMesh, attachedAsAChildOf,
                    withName="PneumaticCavity", withValue=0, withValueType="volumeGrowth"):
    
    """Creates and adds a pneumatic constraint.
    
    The constraint apply to a parent mesh.
    
    Args:
        cavityMeshFile (string): path to the cavity mesh (the mesh should be a surfacic mesh, ie only triangles or quads).
        
        withName (string): name of the created node.

        withValue (real): value to apply, default is 0.
        
        withValueType (string): type of the parameter value (volumeGrowth or pressure), default is volumeGrowth.
        
    Structure:
    .. sourcecode:: qml
        Node : {
                name : "PneumaticCavity"
                MeshTopology,
                MechanicalObject,
                SurfacePressureConstraint,
                BarycentricMapping
        }
        
    """
    
    node = getOrCreateTheTemplateNode(attachedAsAChildOf=attachedAsAChildOf,
                                      withName=withName)

    #  This create a new node in the scene. This node is appended to the finger's node.
    pneumatic = node.createChild(withName)
    
    # This create a MeshTopology, a componant loading and holding the topology of the cavity.
    pneumatic.createObject('MeshTopology', name="topology", filename=fromSurfaceMesh)

    # This create a MechanicalObject, a componant holding the degree of freedom of our
    # mechanical modelling. In the case of a cavity actuated with pneumatic, it is a set of positions specifying
    # the points where the pressure is applied.
    pneumatic.createObject('MechanicalObject', src="@topology")

    # Create a SurfacePressureConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last indice is where the pullPoint is connected.
    pneumatic.createObject('SurfacePressureConstraint',
                          value=withValue,
                          valueType=withValueType)
                        
    # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
    # between the cavity's DoFs and the parents's ones so that the pressure applied on the cavity wall will be mapped
    # to the volume structure and vice-versa;
    pneumatic.createObject('BarycentricMapping', name="Mapping", mapForces="false", mapMasses="false")
    
    return pneumatic
    
def createScene(node):
    from stlib.scene import MainHeader
    MainHeader(node, plugins=["SoftRobots"])
    node.createObject('MechanicalObject')

    PneumaticCavity(fromSurfaceMesh="mesh/cube.obj", attachedAsAChildOf=node)
