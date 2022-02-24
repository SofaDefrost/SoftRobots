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
    MainHeader(rootNode,
               plugins=['SofaPython3', 'SoftRobots', 'SofaOpenglVisual'],
               gravity=[0.0, -9810, 0.0])

    ContactHeader(rootNode,
                  alarmDistance=5,
                  contactDistance=1,
                  frictionCoef=0.7)

    rootNode.VisualStyle.displayFlags = "showBehaviors showCollisionModels"

    Floor(rootNode, **floorParam)

    cube = Cube(rootNode, **cubeParam)
    cube.addObject('UncoupledConstraintCorrection')

    for i in range(len(fingersParameters)):
        finger = ElasticMaterialObject(attachedTo= rootNode,
                                        volumeMeshFileNae=	fingersVolumeMesh,
                                        nae =					fingersParameters[i]['name'],
                                        rotatin =				fingersParameters[i]['rotation'],
                                        translatin =			fingersParameters[i]['translation'],
                                        surfaceMeshFileNae=	fingersSurfaceAndCollisionMesh,
                                        collisionMeh =			fingersSurfaceAndCollisionMesh,
                                        withConstran =			True,
                                        surfaceColr =			fingersColor,
                                        poissonRato =			poissonRatioFingers,
                                        youngModuls =			youngModulusFingers,
                                        totalMas =				fingersMass)

        finger.dofs.name ='tetras'
        rootNode.addChild(finger)

        finger.integration.rayleighStiffness = 0.1
        finger.integration.rayleighMass = 0.1

        finger.addObject('BoxROI', name='boxROI', box=fingersParameters[i]['ROIBox'], drawBoxes=True, doUpdate=False)
        finger.addObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness=1e12
                         , angularStiffness=1e12)

        PneumaticCavity(				surfaceMeshFileNme =	fingersCavitySurfaceMesh,
                                        attachedAsAChilOf =	finger,
                                        nme =					'cavity',
                                        rotaton =				fingersParameters[i]['rotation'],
                                        translaton =			fingersParameters[i]['translation'],
                                        initialVaue =			cavitiesInitialValue,
                                        valueType=				'pressure')

    rootNode.addObject(WholeGripperController(node=rootNode))
