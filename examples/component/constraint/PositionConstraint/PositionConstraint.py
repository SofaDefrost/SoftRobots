def createScene(rootnode):

    rootnode.addObject('RequiredPlugin', name='SoftRobots')

    rootnode.addObject('FreeMotionAnimationLoop')
    rootnode.addObject('GenericConstraintSolver', maxIterations=1e4, tolerance=1e-5, printLog=True)
    rootnode.gravity = [0, -9.81, 0]
    rootnode.addObject('VisualStyle', displayFlags='showForceFields')

    body = rootnode.addChild('Body')
    body.addObject('EulerImplicitSolver')
    body.addObject('SparseLDLSolver')
    body.addObject('RegularGridTopology', min=[0, 0, 0], max=[0.5, 1, 0.1], n=[5, 10, 2])
    body.addObject('MechanicalObject')
    body.addObject('UniformMass', totalMass=0.1)
    body.addObject('TetrahedronFEMForceField', poissonRatio=0.3, youngModulus=1e3)
    body.addObject('GenericConstraintCorrection')
    body.addObject('BoxROI', name="boxToFixed", box=[0, 0, 0, 0.5, 0.1, 0.1], drawBoxes=True)
    body.addObject('BoxROI', name="boxToPull", box=[0, 0.9, 0, 0.5, 1, 0.1], drawBoxes=True)
    body.addObject('FixedProjectiveConstraint', indices=body.boxToFixed.indices.linkpath)
    body.addObject('PartialFixedProjectiveConstraint', indices=body.boxToPull.indices.linkpath, fixedDirections=[1, 0, 1])
    body.addObject('PositionConstraint', indices=body.boxToPull.indices.linkpath,
                   valueType="displacement", value=0.5, useDirections=[0, 1, 0])



