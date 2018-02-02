# -*- coding: utf-8 -*-
import os
path = os.path.dirname(os.path.abspath(__file__))+'/data/mesh/'

def createAndAddToNode(rootNode, name="theFinger",
                       rotation="0 0 0", translation="0 0 0",
                       fixingbox='0 0 0 0 0 0', pullpoint="0 0 0"):
    finger = rootNode.createChild(name)
    finger.createObject('EulerImplicit', name='odesolver', firstOrder='1')
    finger.createObject('SparseLDLSolver', name='preconditioner')

    finger.createObject('MeshVTKLoader', name='loader', filename=path+'finger.vtk')
    finger.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    finger.createObject('MechanicalObject', name='tetras', template='Vec3d')

    ## To be properly simulated and to interact with gravity or inertia forces, an object
    ## also needs a mass. You can add a given mass with a uniform distribution for an object
    ## by adding a UniformMass component to the finger node
    finger.createObject('UniformMass', totalmass=' 0.5')

    ## The next component to add is a FEM forcefield which defines how the object reacts
    ## to a loading (i.e. which deformations are created from forces applied onto it).
    ## Here, because the finger is made of silicone, its mechanical behavior is assumed elastic.
    ## This behavior is available via the TetrahedronFEMForceField component.
    finger.createObject('TetrahedronFEMForceField', template='Vec3d',
                        name='FEM', method='large', poissonRatio='0.3',  youngModulus='18000')

    ## Set the ROI of points of the model to fix.
    ## You can either use "BoxROI"...
    finger.createObject('BoxROI', name='ROI1', box='-15 0 0 5 10 15', drawBoxes='true')
    finger.createObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness='1e12')

    finger.createObject('LinearSolverConstraintCorrection')

    #################################################################################
    ## Visualization
    fingerVisu = finger.createChild('visual')

    ## Add to this empty node a rendering model made of triangles and loaded from an stl file.
    fingerVisu.createObject('OglModel', filename=path+"finger.stl",
                            template='ExtVec3f', color="0.0 0.7 0.7")

    ## Add a BarycentricMapping to deform the rendering model to follow the ones of the
    ## mechanical model.
    fingerVisu.createObject('BarycentricMapping')
   
    #################################################################################
    ## Cable
    cable = finger.createChild('cable')
    cable.createObject('MechanicalObject',
    position=(
        " -17.5 12.5 2.5 " +
        " -32.5 12.5 2.5 " +
        " -47.5 12.5 2.5 " +
        " -62.5 12.5 2.5 " +
        " -77.5 12.5 2.5 " +
        " -83.5 12.5 4.5 " +
        " -85.5 12.5 6.5 " +
        " -85.5 12.5 8.5 " +
        " -83.5 12.5 10.5 " +
        " -77.5 12.5 12.5 " +
        " -62.5 12.5 12.5 " +
        " -47.5 12.5 12.5 " +
        " -32.5 12.5 12.5 " +
        " -17.5 12.5 12.5 " ))

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last indice is where the pullPoint is connected.
    cable.createObject('CableConstraint', name="aCable",
                        indices='0 1 2 3 4 5 6 7 8 9 10 11 12 13',
                        pullPoint="0.0 12.5 2.5")

    # This create a BarycentricMapping. A BarycentricMapping is a key element as it will
    # create a bi-directional link between the cable's DoFs and the finger's ones so that movements
    # of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.createObject('BarycentricMapping')


    # This create a PythonScriptController that permits to programatically implement new behavior
    # or interactions using the Python programming langage. The controller is referring to a
    # file named "controller.py".
    cable.createObject('PythonScriptController', 
                        filename="fingercontroller.py", classname="FingerCableController", autoreload="true")

    #################################################################################
    ## Contact                                
    ## Add a collision model 
    contactPart1 = finger.createChild('contactPart1')

    ## 1- Load the surface mesh for the collision 
    contactPart1.createObject('MeshSTLLoader', name="loader", filename=path+"fingerCollision_part1.stl")
    contactPart1.createObject('Mesh', src="@loader")
    contactPart1.createObject('MechanicalObject')

    # 2- Add a collision model. These three components (Point, Line, Triangle) have to be used together.
    #    Other collision model exist (for example SphereModel) 
    #    Collision model of the same group won't collide.
    contactPart1.createObject('PointModel', group="1")
    contactPart1.createObject('LineModel', group="1")
    contactPart1.createObject('TriangleModel', group="1")

    # 3- Add a mapping to link the collision model to the mechanics
    contactPart1.createObject('BarycentricMapping', mapForces="false", mapMasses="false")

    contactPart2 = finger.createChild('contactPart2')
    contactPart2.createObject('MeshSTLLoader', name="loader", filename=path+"fingerCollision_part2.stl")
    contactPart2.createObject('Mesh', src="@loader")
    contactPart2.createObject('MechanicalObject')
    contactPart2.createObject('Point', group="2")
    contactPart2.createObject('Line', group="2")
    contactPart2.createObject('Triangle', group="2")
    contactPart2.createObject('BarycentricMapping', mapForces="false", mapMasses="false")
