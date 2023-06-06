import params


def addHeader(node):

    node.addObject('RequiredPlugin', name='SoftRobots')
    node.addObject('RequiredPlugin', name='BeamAdapter')
    node.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                            "Sofa.Component.Collision.Detection.Algorithm",
                            # Needed to use components BVHNarrowPhase, BruteForceBroadPhase, DefaultPipeline
                            "Sofa.Component.Collision.Detection.Intersection",  # Needed to use components LocalMinDistance
                            "Sofa.Component.Collision.Response.Contact",  # Needed to use components DefaultContactManager
                            "Sofa.Component.Constraint.Lagrangian.Correction",  # Needed to use components GenericConstraintCorrection
                            "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                            "Sofa.Component.IO.Mesh",  # Needed to use components MeshOBJLoader
                            "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                            "Sofa.Component.Mass",  # Needed to use components UniformMass
                            "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                            "Sofa.Component.Setting",  # Needed to use components BackgroundSetting
                            "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                            "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                            "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
                        ])
    node.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels '
                                               'hideBoundingCollisionModels showForceFields '
                                               'showInteractionForceFields hideWireframe')

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
    node.addObject('DefaultContactManager', response="FrictionContactConstraint")
    node.addObject('LocalMinDistance', alarmDistance=5, contactDistance=1)
    node.addObject('BackgroundSetting', color=[0, 0, 0, 1])
