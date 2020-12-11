# -*- coding: utf-8 -*-

import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__))+'/'
pathMesh = os.path.dirname(os.path.abspath(__file__))+'/mesh/'


def createScene(rootNode):

                rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SoftRobots.Inverse SofaOpenglVisual SofaSparseSolver')
                rootNode.addObject('VisualStyle', displayFlags="showVisualModels hideBehaviorModels hideCollisionModels \
                                        hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe")

                rootNode.addObject('FreeMotionAnimationLoop')

                rootNode.addObject('QPInverseProblemSolver', epsilon=1e-1, maxIterations=1000, tolerance=1e-14)

                rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765])
                rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
                rootNode.findData('gravity').value=[0, 0, -981.0]
                rootNode.findData('dt').value=0.01



                ##########################################
                # Effector goal for interactive control  #
                ##########################################
                goal = rootNode.addChild('goal')
                goal.addObject('EulerImplicitSolver', firstOrder=True)
                goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
                goal.addObject('MechanicalObject', name='goalMO',
                        position=[0, 0, 5],
                        showObject=True,
                        showObjectScale=1,
                        drawMode=1)
                goal.addObject('DataVariationLimiter', name="stabilizer", listening=True, input="@goalMO.position")
                goal.addObject('MechanicalObject', name='goalMOStabilized',
                        position='@stabilizer.output',
                        showObject=True,
                        showObjectScale=1,
                        drawMode=1)
                goal.addObject('UncoupledConstraintCorrection')


                ##########################################
                # FEM Model                              #
                ##########################################
                accordion = rootNode.addChild('accordion')
                accordion.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=0.1, rayleighMass=0.1)
                accordion.addObject('SparseLDLSolver')

                accordion.addObject('MeshVTKLoader', name='loader', filename=pathMesh+'Accordion.vtu', rotation=[90, 0, 0])
                accordion.addObject('TetrahedronSetTopologyContainer', src='@loader')
                accordion.addObject('TetrahedronSetTopologyModifier')

                accordion.addObject('MechanicalObject', name='tetras', template='Vec3')
                accordion.addObject('UniformMass', totalMass=0.030)
                accordion.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,  youngModulus=500)

                accordion.addObject('BoxROI', name='ROI1', box=[-2, -2, 0, 2, 2, 0.5], drawBoxes=True)
                accordion.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)

                accordion.addObject('LinearSolverConstraintCorrection')


                ##########################################
                # Effector                               #
                ##########################################

                effector = accordion.addChild('effector')
                effector.addObject('MechanicalObject', name="effectorPoint",
                        position=[0, 0, 5])
                effector.addObject('PositionEffector', template='Vec3',
                        indices=0,
                        effectorGoal="@../../goal/goalMOStabilized.position",
                        useDirections=[1, 1, 1])
                effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)


                ##########################################
                # Cables                                 #
                ##########################################

                cables = accordion.addChild('cables')
                cables.addObject('MechanicalObject', name="cablesPoint",
                        position=[
                                [1.5, 0, 0.5   ],
                                [1.5, 0, 1.5   ],
                                [1.5, 0, 2.5   ],
                                [1.5, 0, 3.5   ],
                                [1.5, 0, 4.5   ],

                                [0, -1.5, 0.5   ],
                                [0, -1.5, 1.5   ],
                                [0, -1.5, 2.5   ],
                                [0, -1.5, 3.5   ],
                                [0, -1.5, 4.5   ],

                                [-1.5, 0, 0.5   ],
                                [-1.5, 0, 1.5   ],
                                [-1.5, 0, 2.5   ],
                                [-1.5, 0, 3.5   ],
                                [-1.5, 0, 4.5   ]])
                cables.addObject('CableActuator', template='Vec3',
                        name="cable1",
                        indices=list(range(5)),
                        pullPoint=[1.5, 0, 0],
                        minForce=0,
                        maxPositiveDisp=1.5
                        )
                cables.addObject('CableActuator', template='Vec3',
                        name="cable2",
                        indices=list(range(5,10)),
                        pullPoint=[0, -1.5, 0],
                        minForce=0,
                        maxPositiveDisp=1.5
                        )
                cables.addObject('CableActuator', template='Vec3',
                        name="cable3",
                        indices=list(range(10,15)),
                        pullPoint=[-1.5, 0, 0],
                        minForce=0,
                        maxPositiveDisp=1.5
                        )
                cables.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

                ##########################################
                # Pressure                               #
                ##########################################
                cavity = accordion.addChild('cavity')
                cavity.addObject('MeshSTLLoader', name='loader', filename=pathMesh+'Cavity.stl')
                cavity.addObject('MeshTopology', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                cavity.addObject('SurfacePressureActuator', template='Vec3', name="pressure",
                    triangles='@topo.triangles',
                    minPressure=0,
                    maxVolumeGrowth=2)

                cavity.addObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False)

                ##########################################
                # Visualization                          #
                ##########################################
                accordionVisu = accordion.addChild('visu')
                accordionVisu.addObject('MeshObjLoader', filename=pathMesh+"Spring_Cartoon_Body.obj", name="loader")
                accordionVisu.addObject('OglModel', src="@loader", putOnlyTexCoords=True, normals=0, translation=[0, -2.5, 2.5])
                accordionVisu.addObject('BarycentricMapping')


                return rootNode
