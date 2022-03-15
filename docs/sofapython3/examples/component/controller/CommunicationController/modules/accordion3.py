import os
path = os.path.dirname(os.path.abspath(__file__))+'/../mesh/'


def addAccordion(node, inverse=False):

    accordion = node.addChild('accordion')
    accordion.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=0.1, rayleighMass=0.1)
    accordion.addObject('SparseLDLSolver')
    accordion.addObject('MeshVTKLoader', name='loader', filename=path+'Springy.vtk')
    accordion.addObject('MeshTopology', src='@loader')
    accordion.addObject('MechanicalObject', name='tetras', template='Vec3')
    accordion.addObject('UniformMass', totalMass=0.030)
    accordion.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=500)
    accordion.addObject('BoxROI', name='ROI1', box=[-2, -2, 0, 2, 2, 0.5], drawBoxes=True)
    accordion.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)
    accordion.addObject('LinearSolverConstraintCorrection')

    # Pressure
    cavity = accordion.addChild('cavity')
    cavity.addObject('MeshSTLLoader', name='loader', filename=path+'Springy_Cavity.stl')
    cavity.addObject('MeshTopology', src='@loader')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureActuator' if inverse else 'SurfacePressureConstraint', template='Vec3', name="pressure")
    cavity.addObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False)

    # Cables
    cables = accordion.addChild('cables')
    cables.addObject('MechanicalObject', name="cablesPoint",
                        position=[[1.5, 0, 0.5+i] for i in range(5)]
                                 +[[0, -1.5, 0.5+i] for i in range(5)]
                                 +[[-1.5, 0, 0.5+i] for i in range(5)])
    for i in range(3):
        cables.addObject('CableActuator' if inverse else 'CableConstraint', template='Vec3',
                            name="cable"+str(i+1),
                            indices=list(range(i*5,5*(i+1))),
                            pullPoint=[[1.5, 0, 0],[0, -1.5, 0],[-1.5, 0, 0]][i])
    cables.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    # Visualization
    accordionVisu = accordion.addChild('visu')
    accordionVisu.addObject('MeshSTLLoader', filename=path+"Springy.stl", name="loader")
    accordionVisu.addObject('OglModel', src="@loader", color=[0.4, 0.4, 0.4, 0.5])
    accordionVisu.addObject('BarycentricMapping')

    return accordion
