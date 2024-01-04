# -*- coding: utf-8 -*-

# STLIB IMPORT
from stlib3.scene import MainHeader
from stlib3.scene import ContactHeader
from stlib3.physics.rigid import Floor, Cube
from stlib3.physics.deformable import ElasticMaterialObject

# SOFTROBOTS IMPORT
from softrobots.actuators import PullingCable, PneumaticCavity

# CONTROLLER IMPORT
from wholeGripperController import WholeGripperController

# ARGUMENTS IMPORT
from param import *


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')  # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')  # Needed to use components [LocalMinDistance]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')  # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')  # Needed to use components [RuleBasedContactManager]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [LinearSolverConstraintCorrection,UncoupledConstraintCorrection]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver')  # Needed to use components [GenericConstraintSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')  # Needed to use components [BoxROI]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative')  # Needed to use components [CGLinearSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')  # Needed to use components [BarycentricMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')  # Needed to use components [RigidMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring')  # Needed to use components [RestShapeSpringsForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')  # Needed to use components [TetrahedronSetTopologyContainer,TriangleSetTopologyContainer]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]  
    
    MainHeader(rootNode,
               plugins=['SofaPython3', 'SoftRobots'],
               gravity=[0.0, -9810, 0.0])

    ContactHeader(rootNode,
                  alarmDistance=5,
                  contactDistance=1,
                  frictionCoef=0.6)

    rootNode.VisualStyle.displayFlags = "showBehaviors showCollisionModels"

    Floor(rootNode, **floorParam)

    cube = Cube(rootNode, **cubeParam)
    cube.addObject('UncoupledConstraintCorrection')

    for i in range(len(fingersParameters)):
        finger = ElasticMaterialObject(attachedTo=rootNode,
                                       volumeMeshFileName=fingersVolumeMesh,
                                       name=fingersParameters[i]['name'],
                                       rotation=fingersParameters[i]['rotation'],
                                       translation=fingersParameters[i]['translation'],
                                       surfaceMeshFileName=fingersSurfaceAndCollisionMesh,
                                       collisionMesh=fingersSurfaceAndCollisionMesh,
                                       withConstrain=True,
                                       surfaceColor=fingersColor,
                                       poissonRatio=poissonRatioFingers,
                                       youngModulus=youngModulusFingers,
                                       totalMass=fingersMass)

        finger.dofs.name = 'tetras'
        rootNode.addChild(finger)

        finger.integration.rayleighStiffness = 0.1
        finger.integration.rayleighMass = 0.1

        finger.addObject('BoxROI', name='boxROI', box=fingersParameters[i]['ROIBox'], drawBoxes=True, doUpdate=False)
        finger.addObject('RestShapeSpringsForceField', points='@../Finger1/boxROI.indices', stiffness=1e12,
                         angularStiffness=1e12)

        PneumaticCavity(surfaceMeshFileName=fingersCavitySurfaceMesh,
                        attachedAsAChildOf=finger,
                        name='Cavity',
                        rotation=fingersParameters[i]['rotation'],
                        translation=fingersParameters[i]['translation'],
                        initialValue=cavitiesInitialValue,
                        valueType='pressure')

    rootNode.addObject(WholeGripperController(node=rootNode))
