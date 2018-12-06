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
from stlib.physics.constraints import SubTopology

# 	SOFTROBOTS IMPORT 
from softrobots.actuators import PullingCable, PneumaticCavity

# 	CONTROLLER IMPORT
from pythonControllers import GripperController

#	ARGUMENTS IMPORT
from param import *


def createScene(rootNode):

	MainHeader(rootNode, 
		plugins=["SofaPython","SoftRobots"],
		gravity=[0.0,-9810,0.0])

	rootNode.getObject('GenericConstraintSolver').findData('maxIterations').value = '100000'
	rootNode.getObject('GenericConstraintSolver').findData('tolerance').value = '1e-12'

	ContactHeader(rootNode,
		alarmDistance=5,
		contactDistance=1,
		frictionCoef=0.7)

	GripperController(rootNode)

	Floor(rootNode, **floorParam)

	Cube(rootNode, **cubeParam)

	# Put treshold in rigibObject construction param ?
	rootNode.getChild('Cube').getObject('Solver').findData('threshold').value = '1e-6' 

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

		finger.getObject('EulerImplicit').findData('rayleighStiffness').value = 0.1
		finger.getObject('EulerImplicit').findData('rayleighMass').value = 0.1

		finger.createObject('BoxROI', name='boxROI', box=fingersParameters[i]['ROIBox'], drawBoxes='true',doUpdate='0')
		finger.createObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness='1e12', angularStiffness='1e12')

		SubTopology(					attachedTo=				finger,
										topologiyContainer=		'container',
										subTopologiyContainer=	subTopoParameters[i]['subTopologiyContainer'],
										atPositions=			subTopoParameters[i]['atPositions'],
										poissonRatio =			poissonRatioFingers,
										youngModulus =			youngModulusStiffLayerFingers-youngModulusFingers) 

		PneumaticCavity(				surfaceMeshFileName =	fingersCavitySurfaceMesh, 
					    				attachedAsAChildOf =	finger,
					    				name =					'cavity',  
										rotation =				fingersParameters[i]['rotation'],
										translation =			fingersParameters[i]['translation'],
					    				initialValue =			cavitiesInitialValue,
					    				valueType=				'pressure')