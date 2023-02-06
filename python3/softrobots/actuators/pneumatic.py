# -*- coding: utf-8 -*-
import Sofa


def getOrAddTheTemplateNode(attachedAsAChildOf=None, attachedTo=None, name=None):
    if attachedTo != None:
        if name != None:
            Sofa.msg_error(attachedTo,
                           "The name parameter cannot be used when a template is attachedTo to an existing node")
            return attachedTo
        if attachedAsAChildOf != None:
            Sofa.msg_error(attachedTo, "Both attachedTo and attachedAsAChildOf are set is not allowed.")
            return attachedTo
        return attachedTo
    return attachedAsAChildOf.addChild(name)


def PneumaticCavity(surfaceMeshFileName=None,
                    attachedAsAChildOf=None,
                    attachedTo=None,
                    name="PneumaticCavity",
                    rotation=[0.0, 0.0, 0.0],
                    translation=[0.0, 0.0, 0.0],
                    uniformScale=1,
                    initialValue=0,
                    valueType="volumeGrowth"):
    """Adds a pneumatic constraint.

    The constraint apply to a parent mesh.

    Args:
        surfaceMeshFileName (string): path to the cavity mesh (the mesh should be a surface mesh, ie only triangles or quads).
        attachedAsAChildOf: default is None
        attachedTo: default is None
        name (string): name of the added node.
        rotation (vec3): default is [0,0,0]
        translation (vec3): default is [0,0,0]
        uniformScale (real): uniform scale, default is 1.
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
    if attachedAsAChildOf is None and attachedTo is None:
        Sofa.msg_error(
            "Your PneumaticCavity isn't link/child of any node, please set the argument attachedTo or attachedAsAChildOf")
        return None

    if surfaceMeshFileName is None:
        Sofa.msg_error("No surfaceMeshFileName specified, please specify one")
        return None

    pneumatic = getOrAddTheTemplateNode(attachedAsAChildOf=attachedAsAChildOf,
                                        attachedTo=attachedTo,
                                        name=name)

    # This add a MeshSTLLoader, a component loading the topology of the cavity.
    if surfaceMeshFileName.endswith(".stl"):
        pneumatic.addObject('MeshSTLLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation,
                            translation=translation, scale=uniformScale)
    elif surfaceMeshFileName.endswith(".obj"):
        pneumatic.addObject('MeshOBJLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation,
                            translation=translation, scale=uniformScale)
    else:
        Sofa.msg_error(
            "Your surfaceMeshFileName extension is not the right one, you have to give a surfacic mesh with .stl or .obj extension")
        return None

    # This adds a MeshTopology, a component holding the topology of the cavity.
    # pneumatic.addObject('MeshTopology', name="topology", filename=surfaceMeshFileName)
    pneumatic.addObject('MeshTopology', name='topology', src='@MeshLoader')

    # This adds a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cavity actuated with pneumatic, it is a set of positions specifying
    # the points where the pressure is applied.
    pneumatic.addObject('MechanicalObject', src="@topology")

    # Add a SurfacePressureConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    pneumatic.addObject('SurfacePressureConstraint',
                        value=initialValue,
                        valueType=valueType)

    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    # between the cavity's DoFs and the parents' ones so that the pressure applied on the cavity wall will be mapped
    # to the volume structure and vice-versa;
    pneumatic.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)
    return pneumatic


# Exemple doesn't work
def createScene(node):
    from stlib3.scene import MainHeader
    MainHeader(node, plugins=["SoftRobots", 'SofaPython3'])
    node.addObject('MechanicalObject')
    PneumaticCavity(surfaceMeshFileName="mesh/cube.obj", attachedAsAChildOf=node)
