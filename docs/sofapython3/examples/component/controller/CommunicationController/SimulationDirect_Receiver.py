# -*- coding: utf-8 -*-
from modules.accordion3 import addAccordion


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject("RequiredPlugin", name='SofaPython3')
    rootNode.addObject('RequiredPlugin', pluginName=[
                                            "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
                                            "Sofa.Component.Constraint.Lagrangian.Correction",
                                            # Needed to use components LinearSolverConstraintCorrection
                                            "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
                                            "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
                                            "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader, MeshVTKLoader
                                            "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
                                            "Sofa.Component.Mass",  # Needed to use components UniformMass
                                            "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
                                            "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
                                            "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
                                            "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
                                            "Sofa.Component.Visual",  # Needed to use components VisualStyle
                                            "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
                                        ])
    rootNode.addObject('VisualStyle', displayFlags="showVisualModels hideBehaviorModels hideCollisionModels \
                            hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe")

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-14)

    rootNode.gravity.value = [0, 0, -981.0]
    rootNode.dt.value = 0.01

    # FEM Model
    accordion = addAccordion(rootNode)

    # For local communication
    communication = accordion.addObject('CommunicationController', name="sub", listening=True, job="receiver",
                                        port=5558, nbDataField=4, pattern=0)
    # Between two different computers, specify the ip address of the sender
    # accordion.createObject('CommunicationController', name="sub", listening=True, job="receiver", port=5558, nbDataField=4, ip="...")

    accordion.cavity.pressure.value.setParent(communication.data1)
    for i in range(3):
        accordion.cables.getObject('cable' + str(i + 1)).findData('value').setParent(communication.findData(
            'data' + str(i + 2)))

    return rootNode
