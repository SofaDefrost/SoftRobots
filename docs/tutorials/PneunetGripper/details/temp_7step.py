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
from stlib.physics.constraints import FixedBox

# 	SOFTROBOTS IMPORT 
from softrobots.actuators import PullingCable, PneumaticCavity

# 	CONTROLLER IMPORT
from pythonControllers import GripperController

####################      USER PARAM       ##########################

# Fingers Position
radius = 70
angle1 = 120*math.pi/180  # Angle between 1st and 2nd finger in radian
angle2 = 240*math.pi/180  # Angle between 1st and 3rd finger in radian

# Fingers Mesh & Shared Parameters
fingersVolumeMesh = 'data/mesh/pneunetCutCoarse.vtk'
fingersSurfaceAndCollisionMesh = 'data/mesh/pneunetCut.stl',
fingersCavitySurfaceMesh = 'data/mesh/pneunetCavityCut.stl'

youngModulusFingers = 500
youngModulusStiffLayerFingers = 1500
poissonRatioFingers = 0.3
fingersMass = 0.04
cavitiesInitialValue = 0.0001

# Fingers Visu
fingersColor = [0.7, 0.7, 0.7, 0.6]

# Paramters for each Fingers
fingersParameters = [
        {
			'withName' : 				'finger1',
			'withRotation' : 			[0.0, 0.0, 0.0],
			'withTranslation' : 		[0.0, 0.0, 0.0],
			'fixedBox' :				[-15, -15, -40,  15, 15, 10]
        },
        {
			'withName' : 				'finger2',
			'withRotation' : 			[360 - angle1*180/math.pi, 0.0, 0.0],
			'withTranslation' : 		[0.0, radius + radius*math.sin(angle1-math.pi/2), radius*math.cos(angle1-math.pi/2)],
			'fixedBox' :				[-15, -15, -40,  15, 15, 10]
        },
        {
			'withName' : 				'finger3',
			'withRotation' : 			[360 - angle2*180/math.pi, 0.0, 0.0],
			'withTranslation' : 		[0.0, radius + radius*math.sin(angle2-math.pi/2), radius*math.cos(angle2-math.pi/2)],
			'fixedBox' :				[-15, -15, -40,  15, 15, 10]
        }
    ]

#####################################################################


def createScene(rootNode):

	MainHeader(rootNode, 
		plugins=["SofaPython","SoftRobots"],
		gravity=[-9810,0.0,0.0])

	ContactHeader(rootNode,
		alarmDistance=5,
		contactDistance=1,
		withFrictionCoef=0.7)

	GripperController(rootNode)

	Floor(rootNode,
		name="Plane",
		withRotation=[0,0,270],
		withTranslation=[-122,0,0],
		withColor=[1.0, 0.0, 1.0],
		isAStaticObject=True,
		withScale=10)

	Cube(rootNode,
		name="Cube",
		withTotalMass=0.0008,
		withScale=21)

	print rootNode.getChild('Cube').getObject('Solver').findData('threshold')#'1e-6' 

	for i in range(len(fingersParameters)):

		finger = ElasticMaterialObject(	attachedTo =			rootNode,
										fromVolumeMesh =		fingersVolumeMesh,
										withName =				fingersParameters[i]['withName'],
										withRotation =			fingersParameters[i]['withRotation'],
										withTranslation =		fingersParameters[i]['withTranslation'],
										withSurfaceMesh =		fingersSurfaceAndCollisionMesh,
										withCollisionMesh =		fingersSurfaceAndCollisionMesh,
										withConstrain =			True,
										withSurfaceColor =		fingersColor,
										withPoissonRatio =		poissonRatioFingers,
										withYoungModulus =		youngModulusFingers,
										withTotalMass =			fingersMass)

		FixedBox(						atPositions=			fingersParameters[i]['fixedBox'],
										applyTo=				finger,
										withVisualization=		True) 

		PneumaticCavity(				fromSurfaceMesh =		fingersCavitySurfaceMesh, 
					    				attachedAsAChildOf =	finger,
					    				withName =				'cavity',  
					    				# we need to change MeshTopology to MeshSTLLoader to be able to rotate/translate & scale 
										# withRotation =		fingersParameters[i]['withRotation'],
										# withTranslation =		fingersParameters[i]['withTranslation'],
					    				withValue =				cavitiesInitialValue)