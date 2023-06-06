# -*- coding: utf-8 -*-
from modules.accordion3 import addAccordion


def createScene(rootNode):
    INVERSE = False  # Option to use the inverse solvers from the plugin SoftRobots.Inverse

    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject("RequiredPlugin", name='SofaPython3')
    rootNode.addObject('RequiredPlugin', pluginName=[
                                        "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                                        "Sofa.Component.Collision.Geometry",  # Needed to use components SphereCollisionModel
                                        "Sofa.Component.Constraint.Lagrangian.Correction",
                                        # Needed to use components LinearSolverConstraintCorrection, UncoupledConstraintCorrection
                                        "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                                        "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                                        "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader, MeshVTKLoader
                                        "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                                        "Sofa.Component.LinearSolver.Iterative",  # Needed to use components CGLinearSolver
                                        "Sofa.Component.Mass",  # Needed to use components UniformMass
                                        "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                                        "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                                        "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                                        "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                                        "Sofa.Component.Visual",  # Needed to use components VisualStyle
                                        "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
                                         ])
    rootNode.addObject('VisualStyle', displayFlags="showVisualModels hideBehaviorModels showCollisionModels \
                            hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe")

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    if INVERSE:
        rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
        rootNode.addObject('QPInverseProblemSolver', epsilon=1e-1, maxIterations=1000, tolerance=1e-14)
    else:
        rootNode.addObject('GenericConstraintSolver', maxIterations=500, tolerance=1e-5)

    rootNode.gravity.value = [0, 0, -981.0]
    rootNode.dt.value = 0.01

    accordion = addAccordion(rootNode, inverse=INVERSE)

    if INVERSE:
        # Effector goal for interactive control
        goal = rootNode.addChild('goal')
        goal.addObject('EulerImplicitSolver', firstOrder=True)
        goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
        goal.addObject('MechanicalObject', name='goalMO', position=[0, 0, 8])
        goal.addObject('SphereCollisionModel', radius=1)
        goal.addObject('UncoupledConstraintCorrection')

        effector = accordion.addChild('effector')
        effector.addObject('MechanicalObject', name="effectorPoint", position=[0, 0, 5])
        effector.addObject('PositionEffector', template='Vec3',
                           indices=0,
                           effectorGoal=goal.goalMO.position.getLinkPath(),
                           useDirections=[1, 1, 1])
        effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    accordion.cavity.pressure.minPressure = 0
    accordion.cavity.pressure.maxVolumeGrowth = 8
    for i in range(3):
        accordion.cables.getObject('cable' + str(i + 1)).minForce = 0
        accordion.cables.getObject('cable' + str(i + 1)).maxPositiveDisp = 1.5

    accordion.addObject('CommunicationController', listening=True, job="sender", port=5558, nbDataField=4, pattern=0,
                        data1="@cavity/pressure.pressure",
                        data2="@cables/cable1.displacement",
                        data3="@cables/cable2.displacement",
                        data4="@cables/cable3.displacement")

    return rootNode
