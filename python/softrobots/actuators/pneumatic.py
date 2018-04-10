# -*- coding: utf-8 -*-
import Sofa

def getOrCreateTheTemplateNode(attachedAsAChildOf=None, attachedTo=None, name=None):
    if attachedTo != None:
        if name != None:
            Sofa.msg_error(attachedTo, "The name parameter cannot be used when a template is attachedTo to an existing node")
            return attachedTo
        if attachedAsAChildOf != None:
            Sofa.msg_error(attachedTo, "Both attachedTo and attachedAsAChildOf are set is not allowed.")
            return attachedTo
        return attachedTo
    return attachedAsAChildOf.createChild(name)

def PneumaticCavity(surfaceMeshFileName=None,
                    attachedAsAChildOf=None,
                    attachedTo=None,
                    name="PneumaticCavity",
                    rotation=[0.0, 0.0, 0.0],
                    translation=[0.0, 0.0, 0.0],
                    uniformScale=1,
                    initialValue=0,
                    valueType="volumeGrowth"):

    """Creates and adds a pneumatic constraint.

    The constraint apply to a parent mesh.

    Args:
        cavityMeshFile (string): path to the cavity mesh (the mesh should be a surfacic mesh, ie only triangles or quads).

        name (string): name of the created node.

        initialValue (real): value to apply, default is 0.

        valueType (string): type of the parameter value (volumeGrowth or pressure), default is volumeGrowth.

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
    if attachedAsAChildOf == None and attachedTo == None:
        Sofa.msg_error("Your PneumaticCavity isn't link/child of any node, please set the argument attachedTo or attachedAsAChildOf")
        return None

    if surfaceMeshFileName == None:
        Sofa.msg_error("No surfaceMeshFileName specified, please specify one")
        return None

    pneumatic = getOrCreateTheTemplateNode(attachedAsAChildOf=attachedAsAChildOf,
                                           attachedTo=attachedTo,
                                           name=name)

    # This create a MeshSTLLoader, a componant loading the topology of the cavity.
    if surfaceMeshFileName.endswith(".stl"):
        pneumatic.createObject('MeshSTLLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation, translation=translation, scale=uniformScale)
    elif surfaceMeshFileName.endswith(".obj"):
        pneumatic.createObject('MeshObjLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation, translation=translation, scale=uniformScale)
    else :
        Sofa.msg_error("Your surfaceMeshFileName extension is not the right one, you have to give a surfacic mesh with .stl or .obj extension")
        return None

    # This create a MeshTopology, a componant holding the topology of the cavity.
    # pneumatic.createObject('MeshTopology', name="topology", filename=surfaceMeshFileName)
    pneumatic.createObject('Mesh', name='topology', src='@MeshLoader')

    # This create a MechanicalObject, a componant holding the degree of freedom of our
    # mechanical modelling. In the case of a cavity actuated with pneumatic, it is a set of positions specifying
    # the points where the pressure is applied.
    pneumatic.createObject('MechanicalObject', src="@topology")

    # Create a SurfacePressureConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    pneumatic.createObject('SurfacePressureConstraint',
                          value=initialValue,
                          valueType=valueType)

    # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
    # between the cavity's DoFs and the parents's ones so that the pressure applied on the cavity wall will be mapped
    # to the volume structure and vice-versa;
    pneumatic.createObject('BarycentricMapping', name="Mapping", mapForces="false", mapMasses="false")
    return pneumatic

# Exemple doesn't work
def createScene(node):
    from stlib.scene import MainHeader
    MainHeader(node, plugins=["SoftRobots"])
    node.createObject('MechanicalObject')
    PneumaticCavity(surfaceMeshFileName="mesh/cube.obj", attachedAsAChildOf=node)
