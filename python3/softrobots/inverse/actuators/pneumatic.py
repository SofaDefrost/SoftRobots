# -*- coding: utf-8 -*-
import Sofa

def PneumaticCavity(surfaceMeshFileName=None,
                    attachedTo=None,
                    name="PneumaticCavity",
                    minPressure=None,
                    maxPressure=None,
                    minVolumeGrowth=None,
                    maxVolumeGrowth=None,
                    maxVolumeGrowthVariation=None,
                    rotation=[0.0, 0.0, 0.0],
                    translation=[0.0, 0.0, 0.0],
                    uniformScale=1):

    """Adds a pneumatic actuation. Should be used in the context of the resolution of an inverse problem: find the actuation that leads to a desired deformation.
    See documentation at: https://project.inria.fr/softrobot/documentation/constraint/surface-pressure-actuator/

    The constraint is applied to a parent mesh.

    Args:
        cavityMeshFile (string): path to the cavity mesh (the mesh should be a surfacic mesh, ie only triangles or quads).

        name (string): name of the added node.

        minPressure:

        maxPressure:

        minVolumeGrowth:

        maxVolumeGrowth:

        maxVolumeGrowthVariation:

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
    if attachedTo == None:
        Sofa.msg_error("Your PneumaticCavity isn't child of any node, please set the argument attachedTo")
        return None

    if surfaceMeshFileName == None:
        Sofa.msg_error("No surfaceMeshFileName specified, please specify one")
        return None

    # This add a component loading the topology of the cavity.
    if surfaceMeshFileName.endswith(".stl"):
        pneumatic.addObject('MeshSTLLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation, translation=translation, scale=uniformScale)
    elif surfaceMeshFileName.endswith(".obj"):
        pneumatic.addObject('MeshObjLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation, translation=translation, scale=uniformScale)
    else :
        Sofa.msg_error("Your surfaceMeshFileName extension is not the right one, you have to give a surfacic mesh with .stl or .obj extension")
        return None

    # This add a MeshTopology, a component holding the topology of the cavity.
    # pneumatic.addObject('MeshTopology', name="topology", filename=surfaceMeshFileName)
    pneumatic.addObject('MeshTopology', name='topology', src='@MeshLoader')

    # This add a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cavity actuated with pneumatic, it is a set of positions specifying
    # the points where the pressure is applied.
    pneumatic.addObject('MechanicalObject', src="@topology")

    # Add a SurfacePressureConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    pneumatic.addObject('SurfacePressureActuator')

    if minPressure != None : pneumatic.minPressure = minPressure
    if maxPressure != None : pneumatic.maxPressure = maxPressure
    if minVolumeGrowth != None : pneumatic.minVolumeGrowth = minVolumeGrowth
    if maxVolumeGrowth != None : pneumatic.maxVolumeGrowth = maxVolumeGrowth
    if maxVolumeGrowthVariation != None : pneumatic.maxVolumeGrowthVariation = maxVolumeGrowthVariation

    # This add a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    # between the cavity's DoFs and the parents's ones so that the pressure applied on the cavity wall will be mapped
    # to the volume structure and vice-versa;
    pneumatic.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)
    return pneumatic
