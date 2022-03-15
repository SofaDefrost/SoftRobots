import params


def addHeader(node):

    node.addObject('RequiredPlugin', name='Sofa', pluginName=['SofaConstraint', 'SofaDeformable', 'SofaImplicitOdeSolver', 'SofaPreconditioner', 'SofaSparseSolver',
                                                              'SofaGeneralRigid', 'SofaLoader', 'SofaOpenglVisual', 'SofaRigid', 'SofaMiscMapping', 'SofaEngine', 'SofaSimpleFem'])
    node.addObject('RequiredPlugin', name='SoftRobots')
    node.addObject('RequiredPlugin', name='BeamAdapter')
    node.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')

    node.dt.value = params.Simulation.dt
    node.gravity.value = params.Simulation.gravity

    node.addObject('DefaultVisualManagerLoop')
    node.addObject('FreeMotionAnimationLoop')
    if params.Simulation.inverse:
        node.addObject('RequiredPlugin', name='SoftRobots.Inverse')
        node.addObject('QPInverseProblemSolver')
    else:
        node.addObject('GenericConstraintSolver', maxIterations=100, tolerance=0.001)
    node.addObject('DefaultPipeline')
    node.addObject('BruteForceBroadPhase')
    node.addObject('BVHNarrowPhase')
    node.addObject('DefaultContactManager', response="FrictionContact")
    node.addObject('LocalMinDistance', alarmDistance=5, contactDistance=1)
    node.addObject('BackgroundSetting', color=[0,0,0,1])
