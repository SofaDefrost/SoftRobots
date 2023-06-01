import params


def addAllCablesArm(node):
    from generation import addCables
    size = params.Arm.nbRibs * params.Arm.nbSection

    for i in range(params.Arm.nbSection):
        addCables(node.getChild('Section' + str(i + 1)), params.Arm.beamThickness[i], i + 1)

    # middle cable for the gripper
    middleCable = node.addChild('MiddleCable')
    positions = [[0, 0, 0]] * size
    middleCable.addObject('MechanicalObject', position=positions)
    if params.Simulation.inverse:
        middleCable.addObject('CableActuator', name="cable", minForce=0, indices=list(range(size)),
                              pullPoint=[0., 0., 0.])
    else:
        middleCable.addObject('CableConstraint', name="cable", indices=list(range(size)), pullPoint=[0., 0., 0.],
                              valueType="displacement")
    middleCable.addObject('RigidMapping', rigidIndexPerPoint=[4 * i + 1 for i in range(size)], mapForces=False,
                          mapMasses=False, applyRestPosition=True)

    return


def addArm(node):
    from generation import generateRibs

    [position, edges, DOF0TransformNode0, DOF1TransformNode1] = generateRibs()

    size = params.Arm.nbRibs * params.Arm.nbSection
    arm = node.addChild("Arm")
    arm.addData(name='nbDofs', value=len(position), type='int')
    arm.addObject('EulerImplicitSolver', firstOrder=True)
    arm.addObject('SparseLDLSolver', name='ldl', template='CompressedRowSparseMatrixMat3x3d')
    arm.addObject('GenericConstraintCorrection')
    arm.addObject('MeshTopology', position=position, edges=edges)
    arm.addObject('MechanicalObject', template='Rigid3', name='dofs', showObject=True, showObjectScale=10)
    arm.addObject('UniformMass', indices=[4 * i + 1 for i in range(size)], totalMass=0.003 * size,
                  showAxisSizeFactor=0.03)  # set mass of each piece that connect the ribs
    arm.addObject('RestShapeSpringsForceField', stiffness=1e12, angularStiffness=1e12, points=0)  # fix base of the arm

    # sub topology to allow different ribs thickness for each section
    for i in range(params.Arm.nbSection):
        section = arm.addChild('Section' + str(i + 1))
        topology = section.addChild("Topology")
        idFirst = size * i * 2
        idLast = size * (i + 1) * 2

        topology.addObject('MeshTopology', position=position, edges=edges[idFirst:idLast])
        topology.addObject('BeamInterpolation', name="BeamInterpolation", defaultYoungModulus=params.Arm.youngModulus,
                           dofsAndBeamsAligned=False, straight=True, crossSectionShape="rectangular",
                           lengthZ=params.Arm.beamWidth,
                           lengthY=params.Arm.beamThickness[i],
                           DOF0TransformNode0=DOF0TransformNode0,
                           DOF1TransformNode1=DOF1TransformNode1)
        topology.addObject('AdaptiveBeamForceFieldAndMass', name="BeamForceField", computeMass=True,
                           massDensity=params.Arm.massDensity)

    visual = arm.addChild('Visual')
    visual.addObject('MeshOBJLoader', filename=params.Simulation.path + '/mesh/robot.obj', name='loader')
    visual.addObject('OglModel', src='@loader', color=[1., 1., 1., 1.])
    visual.addObject('SkinningMapping')

    addAllCablesArm(arm)

    return arm


def addFinger(node, fingerId, translation, rotation):
    visu = [[], [], []]
    cable = [[], [], []]

    finger = node.addChild('Finger' + str(fingerId + 1))
    finger.addObject('EulerImplicitSolver', firstOrder=False)
    finger.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')
    finger.addObject('GenericConstraintCorrection')
    finger.addObject('MeshVTKLoader', name='loader', filename=params.Simulation.path + '/mesh/finger.vtk',
                     rotation=rotation, translation=translation, scale=1000)
    finger.addObject('MeshTopology', src='@loader', name='container')
    finger.addObject('MechanicalObject', name='dofs', template='Vec3')
    finger.addObject('UniformMass', totalMass=0.0219)
    finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                     youngModulus=1.8e6)
    finger.addObject('BoxROI', name='boxROI', box=[-100, -20, -50, 100, 20, 50], drawBoxes=True)
    finger.addObject('RestShapeSpringsForceField', points=finger.boxROI.indices.getLinkPath(), stiffness=1e12)

    visu[fingerId] = finger.addChild('Visualization')
    visu[fingerId].addObject('MeshSTLLoader', name="loader", filename=params.Simulation.path + "/mesh/finger.stl",
                             scale=1000)
    visu[fingerId].addObject('OglModel', src="@loader", filename=params.Simulation.path + "/mesh/finger.stl",
                             color=[1, 1, 1, 1], rotation=rotation, translation=translation)
    visu[fingerId].addObject('BarycentricMapping')

    cable[fingerId] = finger.addChild('Cable')
    cable[fingerId].addObject('MeshTopology', name='mesh', position=params.Gripper.positions)
    cable[fingerId].addObject('MechanicalObject', name='moCable', rest_position='@mesh.position', rotation=rotation)

    return finger


def addGripper(node, middleCable):
    size = params.Arm.nbRibs * params.Arm.nbSection

    gripper = node.addChild('Gripper')
    fingers = []

    for fingerId in range(3):
        rotation = [0, 120. * fingerId, 0]
        translation = [0, -880, 0]
        fingers.append(addFinger(gripper, fingerId, translation, rotation))

    dofsCable = [[], [], []]
    mappedCable = [[], [], []]

    for fingerId in range(3):
        rotation = [0, 120. * fingerId, 0]

        mappedCable[fingerId] = middleCable.addChild('MappedCableFinger' + str(fingerId + 1))
        fingers[fingerId].Cable.addChild(mappedCable[fingerId])

        mappedCable[fingerId].addObject('MeshTopology', name='mesh', position=params.Gripper.positions)
        dofsCable[fingerId] = mappedCable[fingerId].addObject('MechanicalObject', name='moMappedCable' + str(fingerId),
                                                              rest_position='@mesh.position', rotation=rotation)

        mappedCable[fingerId].addObject('DeformableOnRigidFrameMapping',
                                        input2='@../../dofs',
                                        input1=fingers[fingerId].Cable.getLinkPath(),
                                        output='@./moMappedCable' + str(fingerId), globalToLocalCoords='0',
                                        index="69")

        finalCable = mappedCable[fingerId].addChild('FinalCable' + str(fingerId))
        middleCable.addChild(finalCable)

        finalCable.addObject('MechanicalObject', name='dofs', position=[[0, 0, 0]] * (size + 14))
        indexPairs = [[0, 0]] * (size + 14)

        for i in range(size):
            indexPairs[i] = [0, i]

        for i in range(14):
            indexPairs[i + size] = [1, i]

        if not params.Simulation.inverse:
            finalCable.addObject('CableConstraint', name="CableGrasping",
                                 indices=list(range(32)),
                                 pullPoint=[0.0, 0.0, 0.0])

        finalCable.addObject('SubsetMultiMapping', template='Vec3,Vec3',
                             name='SMM' + str(fingerId),
                             input=[middleCable.MechanicalObject.getLinkPath(), dofsCable[fingerId].getLinkPath()],
                             output='@./dofs',
                             indexPairs=indexPairs)
    return gripper


def createScene(rootnode):
    from header import addHeader
    addHeader(rootnode)
    rootnode.addObject('AttachBodyButtonSetting', stiffness=100)

    arm = addArm(rootnode)
    # gripper = addGripper(rootnode, arm.MiddleCable)

    if not params.Simulation.inverse and params.Simulation.armGUI:
        from armGUI import ArmGUI
        rootnode.addObject(ArmGUI(arm=arm))

    if params.Simulation.inverse:
        # The target will be represented by an orange sphere in the scene.
        # You can select the target with the mouse (left click) while pressing the shift key on the keyboard.
        # Once selected you can move the target around and see the robot follow.
        target = rootnode.addChild('Target')
        target.addObject('EulerImplicitSolver', firstOrder=True)
        target.addObject('CGLinearSolver', iterations=10, threshold=1e-5, tolerance=1e-5)
        target.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[0, 880, 0, 0, 0, 0, 1])
        target.addObject('SphereCollisionModel', radius=20)
        target.addObject('UncoupledConstraintCorrection')
        arm.addObject('PositionEffector', template="Rigid3", effectorGoal=target.dofs.position.getLinkPath(),
                      indices=arm.nbDofs.value - 4, useDirections=[1, 1, 1, 0, 0, 0])

    return
