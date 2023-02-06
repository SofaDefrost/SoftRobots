# -*- coding: utf-8 -*-
import Sofa

def getOrAddTheTemplateNode(attachedAsAChildOf=None, attachedTo=None, name=None):
    if attachedTo != None:
        if name != None:
            Sofa.msg_error(attachedTo, "The name parameter cannot be used when a template is attachedTo to an existing node")
            return attachedTo
        if attachedAsAChildOf != None:
            Sofa.msg_error(attachedTo, "Both attachedTo and attachedAsAChildOf are set is not allowed.")
            return attachedTo
        return attachedTo
    return attachedAsAChildOf.addChild(name)

def PneumaticSensor(surfaceMeshFileName=None,
                    attachedAsAChildOf=None,
                    attachedTo=None,
                    name="PneumaticSensor",
                    rotation=[0.0, 0.0, 0.0],
                    translation=[0.0, 0.0, 0.0],
                    uniformScale=1,
                    initialValue=0,
                    valueType="volumeGrowth"):

    """Adds a pneumatic sensor constraint.

    This initializes all the components necessary to implement a virtual volume sensor. For a given geometry this will initialize a cavity that will deform as the mesh deforms. The component calculates the volume, but doesn't apply any constraint, i.e. there is no
    forces coming from a change in pressure for instance. This is the case when in real life air flow sensors are being used.

    The constraint apply to a parent mesh.

    Args:
        cavityMeshFile (string): path to the cavity mesh (the mesh should be a surfacic mesh, ie only triangles or quads).

        name (string): name of the added node.

        initialValue (real): value to apply, default is 0.

        valueType (string): type of the parameter value (volumeGrowth or pressure), default is volumeGrowth.

    Structure:
    .. sourcecode:: qml
        Node : {
                name : "PneumaticSensor"
                MeshTopology,
                MechanicalObject,
                SurfacePressureConstraint,
                BarycentricMapping
        }

    """
    if attachedAsAChildOf == None and attachedTo == None:
        Sofa.msg_error("Your PneumaticSensor isn't link/child of any node, please set the argument attachedTo or attachedAsAChildOf")
        return None

    if surfaceMeshFileName == None:
        Sofa.msg_error("No surfaceMeshFileName specified, please specify one")
        return None

    PneumaticSensor = getOrAddTheTemplateNode(attachedAsAChildOf=attachedAsAChildOf,
                                           attachedTo=attachedTo,
                                           name=name)

    # This add a MeshSTLLoader, a component loading the topology of the cavity.
    if surfaceMeshFileName.endswith(".stl"):
        PneumaticSensor.addObject('MeshSTLLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation, translation=translation, scale=uniformScale)
    elif surfaceMeshFileName.endswith(".obj"):
        PneumaticSensor.addObject('MeshObjLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation, translation=translation, scale=uniformScale)
    else :
        Sofa.msg_error("Your surfaceMeshFileName extension is not the right one, you have to give a surfacic mesh with .stl or .obj extension")
        return None

    # This add a MeshTopology, a component holding the topology of the cavity.
    # PneumaticSensor.addObject('MeshTopology', name="topology", filename=surfaceMeshFileName)
    PneumaticSensor.addObject('MeshTopology', name='topology', src='@MeshLoader')

    # This add a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cavity actuated with PneumaticSensor, it is a set of positions specifying
    # the points where the pressure is applied.
    PneumaticSensor.addObject('MechanicalObject', src="@topology")

    # Add a SurfacePressureConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    #PneumaticSensor.addObject('SurfacePressureConstraint',
    #                      value=initialValue,
    #                      valueType=valueType)

    # This add a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    # between the cavity's DoFs and the parents's ones so that the pressure applied on the cavity wall will be mapped
    # to the volume structure and vice-versa;
    PneumaticSensor.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)
    PneumaticSensor.addObject('SurfacePressureSensor',  triangles='@topology.triangles')
    return PneumaticSensor

def createScene(node):
    from stlib3.scene import MainHeader
    MainHeader(node, plugins=["SoftRobots"])
    node.addObject('MechanicalObject')
    PneumaticSensor(surfaceMeshFileName="mesh/cube.obj", attachedAsAChildOf=node)
