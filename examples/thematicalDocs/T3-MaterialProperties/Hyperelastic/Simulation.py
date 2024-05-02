import Sofa

import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


# Units: cm and kg

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels showForceFields')

    rootNode.dt = 0.001
    rootNode.gravity = [0., 0., -9810]

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    ###############################
    # MECHANICAL MODEL
    ###############################

    robot = rootNode.addChild('robot')
    robot.addObject('EulerImplicitSolver')
    robot.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')
    robot.addObject('MeshVTKLoader', name='loader', filename=path + 'branch.vtu')
    robot.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@loader.tetrahedra',
                    name='container', createTriangleArray=True)
    robot.addObject('TetrahedronSetTopologyModifier')

    robot.addObject('MechanicalObject', name='tetras')
    # UniformMass: Here you can set the mass of the structure
    robot.addObject('UniformMass', totalMass=0.4);
    # TetrahedronFEMForceField:
    # By using this component you choose hyper-elastic model
    # Here you can set the youngModulus to adapt the stiffness of the deformable structurepoissonRatio = 0.2
    youngModulus = 600
    poissonRatio = 0.4

    # StVenantKirchhoff
    mu_ = youngModulus / (2 * (1 + poissonRatio))
    lambda_ = youngModulus * poissonRatio / ((1 - 2 * poissonRatio) * (1 + poissonRatio))
    robot.addObject('TetrahedronHyperelasticityFEMForceField', materialName="StVenantKirchhoff",
                    ParameterSet=str(mu_) + " " + str(lambda_))

    # NeoHookean
    # mu_ = youngModulus/(2*(1+poissonRatio))
    # k0_ = youngModulus/(3*(1-2*poissonRatio))
    # robot.addObject('TetrahedronHyperelasticityFEMForceField', materialName="NeoHookean", ParameterSet=str(mu_) + " " + str(k0_))

    # MooneyRivlin
    # c_1 = 30
    # c_2 = 2*mu_ - c_1
    # robot.addObject('TetrahedronHyperelasticityFEMForceField', materialName="MooneyRivlin", ParameterSet=str(c_1) + " " + str(c_2) + " " + str(k0_))

    robot.addObject('BoxROI', name='boxROI',
                    box=[[-30, 80, 70, 30, 140, 130], [-130, -100, 70, -60, -20, 130], [130, -100, 70, 60, -20, 130]],
                    drawBoxes=True)
    robot.addObject('FixedProjectiveConstraint', indices="@boxROI.indices")

    return rootNode
