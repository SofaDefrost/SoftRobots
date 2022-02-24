# -*- coding: utf-8 -*-
import math

####################      USER PARAM       ##########################

cubeParam = { 	'name' : "Cube",
				'totalMass' : 0.0008,
				'translation' : [15.0,20.0,0.0],
				'uniformScale' : 21}

floorParam = {	'name' : "Plane",
				'color' : [1.0, 0.0, 1.0],
				'isAStaticObject' : True,
				'uniformScale' : 10}


# Fingers Mesh & Shared Parameters
fingersVolumeMesh = 'data/mesh/pneunetCutCoarse.vtk'
fingersSurfaceAndCollisionMesh = 'data/mesh/pneunetCut.stl'
fingersCavitySurfaceMesh = 'data/mesh/pneunetCavityCut.stl'

# Fingers Visu
fingersColor = [0.7, 0.7, 0.7, 0.6]

# Fingers Position
heightInitial = 150
radius = 70
angle1 = 120*math.pi/180  # Angle between 1st and 2nd finger in radian
angle2 = 240*math.pi/180  # Angle between 1st and 3rd finger in radian

youngModulusFingers = 500
youngModulusStiffLayerFingers = 1500
poissonRatioFingers = 0.3
fingersMass = 0.04
cavitiesInitialValue = 0.0001

# Paramters for each Fingers
fingersParameters = [
        {
			'name' : 				'finger1',
			'rotation' : 			[0.0, 0.0, -270.0],
			'translation' : 		[100.0, heightInitial , 0.0],
			'ROIBox' :				[100, heightInitial-10, -20, 70, heightInitial, 20],
        },
        {
			'name' : 				'finger2',
			'rotation' : 			[360 - angle1*180/math.pi, 0.0, 90.0],
			'translation' : 		[0, heightInitial, radius*math.cos(angle1-math.pi/2)],
			'ROIBox' :				[100, heightInitial-10, -20, 70, heightInitial, 20],
        },
        {
			'name' : 				'finger3',
			'rotation' : 			[360 - angle2*180/math.pi, 0.0, 90.0],
			'translation' : 		[0.0, heightInitial, radius*math.cos(angle2-math.pi/2)],
			'ROIBox' :				[100, heightInitial-10, -20, 70, heightInitial, 20],
        }
    ]
