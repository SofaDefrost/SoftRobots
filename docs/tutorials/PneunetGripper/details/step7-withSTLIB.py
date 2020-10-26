# -*- coding: utf-8 -*-

#	STANDARD IMPORT
import Sofa
import math
import os

#	STLIB IMPORT
from stlib.scene import MainHeader
from stlib.scene import ContactHeader
from stlib.physics.rigid import Floor, Cube
from stlib.physics.deformable import ElasticMaterialObject

# 	SOFTROBOTS IMPORT
from softrobots.actuators import PullingCable, PneumaticCavity

# 	CONTROLLER IMPORT
from pythonControllers import GripperController

#	ARGUMENTS IMPORT
from param import *


def createScene(rootNode):

	MainHeader(rootNode,
		plugins=['SofaPython','SoftRobots', 'SofaOpenglVisual'],
		gravity=[0.0, -9810, 0.0])

	ContactHeader(rootNode,
		alarmDistance=5,
		contactDistance=1,
		frictionCoef=0.7)

	GripperController(rootNode)

	Floor(rootNode, **floorParam)

	Cube(rootNode, **cubeParam)

	# Put treshold in rigibObject construction param ?
	rootNode.Cube.Solver.threshold = 1e-6

	for i in range(len(fingersParameters)):

		finger = ElasticMaterialObject(	attachedTo =			rootNode,
										volumeMeshFileName =	fingersVolumeMesh,
										name =					fingersParameters[i]['name'],
										rotation =				fingersParameters[i]['rotation'],
										translation =			fingersParameters[i]['translation'],
										surfaceMeshFileName =	fingersSurfaceAndCollisionMesh,
										collisionMesh =			fingersSurfaceAndCollisionMesh,
										withConstrain =			True,
										surfaceColor =			fingersColor,
										poissonRatio =			poissonRatioFingers,
										youngModulus =			youngModulusFingers,
										totalMass =				fingersMass)

		finger.integration.rayleighStiffness = 0.1
		finger.integration.rayleighMass = 0.1

		finger.createObject('BoxROI', name='boxROI', box=fingersParameters[i]['ROIBox'], drawBoxes=True, doUpdate=False)
		finger.createObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness=1e12, angularStiffness=1e12)

		PneumaticCavity(				surfaceMeshFileName =	fingersCavitySurfaceMesh,
					    				attachedAsAChildOf =	finger,
					    				name =					'cavity',
										rotation =				fingersParameters[i]['rotation'],
										translation =			fingersParameters[i]['translation'],
					    				initialValue =			cavitiesInitialValue,
					    				valueType=				'pressure')
