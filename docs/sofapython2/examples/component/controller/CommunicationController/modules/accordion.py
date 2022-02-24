import os
path = os.path.dirname(os.path.abspath(__file__))+'/../mesh/'


def addAccordion(node, inverse=False):

    accordion = node.createChild('accordion')
    accordion.createObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=0.1, rayleighMass=0.1)
    accordion.createObject('SparseLDLSolver')
    accordion.createObject('MeshVTKLoader', name='loader', filename=path+'Springy.vtk')
    accordion.createObject('MeshTopology', src='@loader')
    accordion.createObject('MechanicalObject', name='tetras', template='Vec3')
    accordion.createObject('UniformMass', totalMass=0.030)
    accordion.createObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=500)
    accordion.createObject('BoxROI', name='ROI1', box=[-2, -2, 0, 2, 2, 0.5], drawBoxes=True)
    accordion.createObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)
    accordion.createObject('LinearSolverConstraintCorrection')

    # Pressure
    cavity = accordion.createChild('cavity')
    cavity.createObject('MeshSTLLoader', name='loader', filename=path+'Springy_Cavity.stl')
    cavity.createObject('MeshTopology', src='@loader')
    cavity.createObject('MechanicalObject', name='cavity')
    cavity.createObject('SurfacePressureActuator' if inverse else 'SurfacePressureConstraint', template='Vec3', name="pressure")
    cavity.createObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False)

    # Cables
    cables = accordion.createChild('cables')
    cables.createObject('MechanicalObject', name="cablesPoint",
                        position=[[1.5, 0, 0.5+i] for i in range(5)]
                                 +[[0, -1.5, 0.5+i] for i in range(5)]
                                 +[[-1.5, 0, 0.5+i] for i in range(5)])
    for i in range(3):
        cables.createObject('CableActuator' if inverse else 'CableConstraint', template='Vec3',
                            name="cable"+str(i+1),
                            indices=range(i*5,5*(i+1)),
                            pullPoint=[[1.5, 0, 0],[0, -1.5, 0],[-1.5, 0, 0]][i])
    cables.createObject('BarycentricMapping', mapForces=False, mapMasses=False)

    # Visualization
    accordionVisu = accordion.createChild('visu')
    accordionVisu.createObject('MeshSTLLoader', filename=path+"Springy.stl", name="loader")
    accordionVisu.createObject('OglModel', src="@loader", color=[0.4, 0.4, 0.4, 0.5])
    accordionVisu.createObject('BarycentricMapping')

    return accordion
