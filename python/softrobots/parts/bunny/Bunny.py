import os
meshpath = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def createBunny(Node, Translation=[0,0,0],ControlType='PressureConstraint', YoungModulus=18000):

    #Bunny
    BoundingBox=[-5, -6, -5,  5, -4.5, 5] + [Translation,Translation]
    Bunny = Node.createChild('Bunny')
    Bunny.createObject('EulerImplicit', name='odesolver')
    Bunny.createObject('SparseLDLSolver', name='preconditioner')
    Bunny.createObject('MeshVTKLoader', name='loader', filename=meshpath+'Hollow_Stanford_Bunny.vtu', translation=Translation)
    Bunny.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    Bunny.createObject('TetrahedronSetTopologyModifier')
    Bunny.createObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
    Bunny.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')
    Bunny.createObject('MechanicalObject', name='tetras', template='Vec3d')
    Bunny.createObject('UniformMass', totalmass='0.5')
    Bunny.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', poissonRatio='0.3',  youngModulus=YoungModulus)
    Bunny.createObject('BoxROI', name='boxROI', box=BoundingBox, drawBoxes='true', position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
    Bunny.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')
    Bunny.createObject('LinearSolverConstraintCorrection')
                
    cavity = Bunny.createChild('cavity')
    cavity.createObject('MeshObjLoader', name='loader', filename=meshpath+'Hollow_Bunny_Body_Cavity.obj', translation=Translation)
    cavity.createObject('Mesh', src='@loader', name='topo')
    cavity.createObject('MechanicalObject', name='cavity')
    
    if(ControlType=='PressureConstraint'):
        cavity.createObject('SurfacePressureConstraint', triangles='@topo.triangles', value='40')
    elif (ControlType=='VolumeConstraint'):
        cavity.createObject('SurfacePressureConstraint', triangles='@topo.triangles', value='40')
        
    cavity.createObject('BarycentricMapping', name='mapping',  mapForces='false', mapMasses='false')
    
    BunnyVisu = Bunny.createChild('visu')
    BunnyVisu.createObject('TriangleSetTopologyContainer', name='container')
    BunnyVisu.createObject('TriangleSetTopologyModifier')
    BunnyVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
    BunnyVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
    BunnyVisu.createObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container", output="@container")
    BunnyVisu.createObject('OglModel', template='ExtVec3f', color='0.3 0.2 0.2 0.6', translation=Translation)
    BunnyVisu.createObject('IdentityMapping')
    return Bunny
                