# Simulation

The simulation is done using the SOFA Open Source Framework and the "Soft-Robots" Plugin dedicated for Real-time simulation of Soft Robots. To define the simulation, a Python scene file is created and fed as input to SOFA. In the following, we will describe, step by step, the creation of scene file that perfom the simulation of the soft Pneunet gripper. A SOFA scene is an ordered tree of nodes (ex: ```finger.```), with parent/child relationship (set up by ```node.createChild()```). Each node has one or a few components (set up with ```node.createObject()```). Every node and component has a name and a few features. The main node at the top of the tree is called "rootNode".

## Volumetric Meshing and Loading

To be able to simulate the soft robot, the first step is to discretise the soft robot in space, by creating a volumetric mesh, typically with tetrahedra. This can be done with any meshing tool such as Gmsh or CGAL. In this example, we use Gmsh. This will generate a vtk file containing all the information about postion of the nodes and connectivity between them through the tetrahedra.


[Meshed finger](data/images/PneuNets-gripper_mesh.png)

In a SOFA scene the mesh is loaded using the loader component:
```python
finger.createObject('MeshVTKLoader', name='loader', filename='data/mesh/pneunetCutCoarse.vtk')
```

This mesh is then stored in a ```Mesh```  component (linked to the loader through its ```src``` attribute), and a MechanicalObject component is created to store the degrees of freedom of the robot (which are the positions of all the nodes in the mesh)

```python
finger.createObject('Mesh', src='@loader', name='container')
finger.createObject('MechanicalObject', name='tetras', template='Vec3d', rx='0', dz='0')
```
[Step1](step1-meshLoading.pyscn)
## Constitutive law of the material and Mass

### Main Body

Next, we need to define what kind of material we are going to simulate, and this is done by adding a ForceField component, which describes what internal forces are created when the object is deformed. In particular, this will define how soft or stiff the material is, if it has an elastic or more complex behaviour (Hyperelastic, plastic, etc...). In this example, we use the TetrehedronFEMForceField component which corresponds to an elastic material deformation but with large rotations. Attributes such as Young's Modulus and Poisson's ratio can be set within this component:
```python
finger.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus='500')
```
The vertexMass of the material can be defined with the UniformMass component, which assumes a uniform distribution of the vertexMass inside the body:
```python
finger.createObject('UniformMass', totalMass='0.0008')
```
Note that by default, the gravity is defined as "0 -9.81 0". You can redefine it with the following command in the rootNode:
```python
rootNode.findData('gravity').value='-9810 0 0';
```
This corresponds to a gravity defined along the x axis, assuming the length unit is millimeters.

[Step2](step2-forceFieldAndMass.pyscn)

### Stiff layer

To define the constitutive law of the stiff layer, we will create a new node and define a new ForceField with stiffer parameters only on the points which constitute the layer. To easily define the indices of the points which will be selected, we use the boxROI component wich allows to define a box that will contain all the points of the layer (refered by the node "```modelSubTopo```"). The box component contains 	successively  the extreme coordinates along x, y and z

```python
finger.createObject('BoxROI', name='boxROISubTopo', box='-100 22.5 -8 -19 28 8')
modelSubTopo = finger.createChild('modelSubTopo')
modelSubTopo.createObject('Mesh', position='@loader.position', tetrahedra="@boxROISubTopo.tetrahedraInROI", name='container')
modelSubTopo.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus='1500')
```

[Step3](step3-stiffLayer.pyscn)

## Boundary Conditions

To fix the finger in space, it is necessary to define boundary conditions on some parts of the object. This can be done in several ways in SOFA. In this case, we use the component RestShapeSpringsForceField, which creates springs between the current position of a part of the body and its initial position. The stiffness of these springs can be adjusted. In particular, setting a very large stiffness will be equivalent to fixing the points in space.
To easily define the indices of the points which will be fixed, we use the boxROI component:

```python
finger.createObject('BoxROI', name='boxROI', box='-10 0 -20 0 30 20')
finger.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12', angularStiffness='1e12')
```

[Step4](step4-boundaryConditions.pyscn)

## Implicit Time Integration and Matrix Solver

Now that all the minimal elements of the scene have been described, it is already possible to simulate the deformation of the finger. To do that, we need to define a time integration scheme, which will define the system to be solved at each time step of the simulation, as well as a matrix solver, to solve that system and compute the updated velocities and positions of the nodes of the robot. In this case, we choose to use an Implicit Euler Scheme and a sparse direct Solver base on an LDL matrix decomposition:

```python
finger.createObject('EulerImplicit', name='odesolver')
finger.createObject('SparseLDLSolver', name='directSolver')
```
With the scene in this state, not much will happen in the simulation, merely a small deformation due to gravity on the finger. Indeed, we did not include the pneumatic actuator yet.

[Step5](step5-timeIntegrationAndMatrixSolver.pyscn)

## Pneumatic Actuator and Python script controller

In this section, we will introduce a pneumatic actuator that will allow to interactively simulate the inflation of the finger's cavity. To do that, we first create a node that will take care of this task. Then, the surface mesh of the cavity has to be loaded using a mesh loader and the position are stored in a Mesh component which is a container. A MechanicalObject is created to store the degrees of freedom of the cavity which will be deforming during the simulation. The actuator is created with the component SurfacePressureConstraint which is directly associated to the mesh container through the attribute "triangles". The actuation can be defined either by pressure or volume growth. Finally, a BarycentricMapping component is created to map the deformation of the cavity mesh to the mesh of the finger:

```python
cavity = finger.createChild('cavity')
cavity.createObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCavityCut.stl')
cavity.createObject('Mesh', src='@loader', name='cavityMesh')
cavity.createObject('MechanicalObject', name='cavity')
cavity.createObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0.0001", triangles='@topo.triangles', valueType="pressure")
cavity.createObject('BarycentricMapping', name='mapping')
```

To interactively inflate the cavity, we use a PythonScriptController, which is a component referring to a python file performing some actions at the initialisation or during the simulation:
```python
rootNode.createObject('PythonScriptController', filename="controllerGripper.py", classname="controller")
```
In this case, the controller allows to interactively inflate the cavity by pressing ctrl + '+' and deflate it by pressing ctrl + '-'.


## Solving the constraints

 To solve the constraints, such as the one define by the pressure actuator, we have to add to the rootNode the component FreeMotionAnimationLoop that will build up the system including constraints. The component GenericConstraintSolver will also be added to solve the constraints problem. Finally, we add the component LinearConstraintCorrection to the finger Node to take into account the correction due to the cavity constraint to the velocity and position:
```python
rootNode.createObject('FreeMotionAnimationLoop')
rootNode.createObject('GenericConstraintSolver', maxIterations="10000", tolerance="1e-3")
```

```python
finger.createObject('LinearSolverConstraintCorrection', solverName='directSolver')
```
With all these components, the scene is now runable and can be used to inflate and deflate the finger. 

[Real images](data/images/PneuNets-gripper_OneFingerBendingAll.png)

[Step6](step6-pneumaticActuatorAndPythonScriptController.pyscn)

## Creating the gripper

### Add fingers

Now, to create an actual gripper, we just need to create two more fingers and define their positions. To do that, you can use the same mesh files and use the 'translation' and 'rotation' attributes in the mesh loaders. Check the scene file in the appendix for more details.

### Add an object to grab and a plane to put it on

In this scene, we create the object to grab and define it as rigid. This implies creating a new node, adding a time integration and a solver component, defining a vertexMass, and defining constraint corrections that will be used later for collisons. 

```python
cube = rootNode.createChild('cube')
cube.createObject('EulerImplicit', name='odesolver')
cube.createObject('SparseLDLSolver', name='linearSolver')
cube.createObject('MechanicalObject', template="Rigid3", scale="4", position='-23 16 0 0 0 0 1')
cube.createObject('UniformMass', vertexMass='0.0008  74088  0.2352 0 0  0 0.2352 0  0 0 0.2352')                cube.createObject('UncoupledConstraintCorrection')
```

We also need to define a plane, using the predefined meshes of SOFA:

```python
planeNode = rootNode.createChild('Plane')
planeNode.createObject('MeshObjLoader', name='loader', filename="mesh/floorFlat.obj", triangulate="true", rotation="0 0 270", scale =10, translation="-122 0 0")
planeNode.createObject('Mesh', src="@loader")
planeNode.createObject('MechanicalObject', src="@loader")
```
In this case, the file "floorFlat.obj" is defined in a shared directory of SOFA that is accessible in any SOFA scene.

### Add collisions

As it stands, objects would go through each other as if they were ghosts. It is therefore necessary to handle collisions. This is simply done in SOFA by adding a collision model. In the case of 3-dimensional objects described with triangles and tetrahedra, this is typically done by adding the components 'Triangle', 'Line' and 'Point', that define the elements on the surface of the objects.

#### Plane
 The case of the plane is the most straignthforward: it is described by a surface and the components of collision can be directly added to its node, specifying it is not moving during the simulation:
 
```python
planeNode.createObject('Triangle', simulated="0", moving="0")
planeNode.createObject('Line', simulated="0", moving="0")
planeNode.createObject('Point', simulated="0", moving="0")
```

#### Object to grab

For the object to grab, we said it as going to be rigid, but we did not define its shape yet. This is needed to define a collision model. Hence, we create a child node that will load this shape (predefined mesh available in SOFA), which we choose to be a cube and we add the collision model. It is also necessary to add a rigid mapping which will map the degrees of freedom of the cube mesh to a rigid SOFA component.
```python
cubeCollis = cube.createChild('cubeCollis')
cubeCollis.createObject('MeshObjLoader', name="loader", filename="mesh/smCube27.obj", triangulate="true",  scale="6")
cubeCollis.createObject('Mesh', src="@loader")
cubeCollis.createObject('MechanicalObject', translation='0 0 0')
cubeCollis.createObject('Triangle')
cubeCollis.createObject('Line')
cubeCollis.createObject('Point')
cubeCollis.createObject('RigidMapping')
```
#### Fingers

The procedure is similar with the fingers: in a child node, we load a mesh representing the surface of the fingers and apply the collision model on it. Note that the nodes of this surface mesh need not match the ones of the associated volumetric mesh. Indeed, a barycentric mapping is used to map those collision degrees of freedom to the volumetric mesh ones:

```python
collisionFinger = finger.createChild('collisionFinger')
collisionFinger.createObject('MeshSTLLoader', name='loader', filename=path+'pneunetCut.stl', translation = translateFinger)
collisionFinger.createObject('Mesh', src='@loader', name='topo')
collisionFinger.createObject('MechanicalObject', name='collisMech')
collisionFinger.createObject('Triangle', selfCollision="false")
collisionFinger.createObject('Line',selfCollision="false")
collisionFinger.createObject('Point', selfCollision="false")
collisionFinger.createObject('BarycentricMapping')
```


## That's it, you can now run the simulation and try to grab the cube!

In this case, the controller allows to inflate all the cavities by pressing ctrl + '+' and deflate them by pressing ctrl + '-'.
The gripper can be moved up, down, left and right by pressing respectively  ctrl + Up, Down, Left, Right. It can be turned clockwise and anticlockwise by pressing ctrl + 'a' and ctrl + 'q'.

[Step7](step7-grabTheCube.pyscn)
If you want to play around, you can add components OglModel for every object to have a visual representation that suits you.

![Real images](data/images/PneuNets-gripper_inAction.png)

## Appendix

Here is the final SOFA scene simulation of the soft pneunet Gripper
```python
import Sofa
import math
import os

youngModulusFingers = 500
youngModulusStiffLayerFingers = 1500
poissonRatioFingers = 0.3
fingersMass = 0.04

radius = 70
angle1 = 120*math.pi/180  # Angle between 1st and 2nd finger in radian
angle2 = 240*math.pi/180  # Angle between 1st and 3rd finger in radian
translateFinger1 = "0 0 0"
translateFinger2 = "0 " + str(radius + radius*math.sin(angle1-math.pi/2)) + " " + str(radius*math.cos(angle1-math.pi/2))
translateFinger3 = "0 " + str(radius + radius*math.sin(angle2-math.pi/2)) + " " + str(radius*math.cos(angle2-math.pi/2))

def createScene(rootNode):

                rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
                rootNode.createObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
                rootNode.findData('gravity').value='-9810 0 0';
                rootNode.createObject('FreeMotionAnimationLoop')
                rootNode.createObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")
                rootNode.createObject('CollisionPipeline', verbose="0")
                rootNode.createObject('BruteForceDetection', name="N2")
                rootNode.createObject('CollisionResponse', response="FrictionContact", responseParams="mu=0.7")
                rootNode.createObject('LocalMinDistance', name="Proximity", alarmDistance="5", contactDistance="1", angleCone="0.01")

		rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
                rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")
                rootNode.createObject('PythonScriptController', filename="pythonControllers/wholeGripperController.py", classname="controller")

                planeNode = rootNode.createChild('Plane')
                planeNode.createObject('MeshObjLoader', name='loader', filename="data/mesh/floorFlat.obj", triangulate="true", rotation="0 0 270", scale =10, translation="-122 0 0")
                planeNode.createObject('Mesh', src="@loader")
                planeNode.createObject('MechanicalObject', src="@loader")
                planeNode.createObject('Triangle', simulated="0", moving="0")
                planeNode.createObject('Line', simulated="0", moving="0")
                planeNode.createObject('Point', simulated="0", moving="0")
                planeNode.createObject('OglModel',name="Visual", fileMesh="data/mesh/floorFlat.obj", color="1 0 0 1",rotation="0 0 270", scale =10, translation="-122 0 0")

                cube = rootNode.createChild('cube')
                cube.createObject('EulerImplicit', name='odesolver')
                cube.createObject('SparseLDLSolver', name='linearSolver')
                cube.createObject('MechanicalObject', template="Rigid3", scale="4", position='-23 16 0 0 0 0 1')#, dx="47.0", dy="10", dz="8", rx="10" ,ry="10")
                #cube.createObject('UniformMass', vertexMass='0.0008  74088  0.2352 0 0  0 0.2352 0  0 0 0.2352')
                cube.createObject('UniformMass', vertexMass='0.0008')
                cube.createObject('UncoupledConstraintCorrection')
                
                #collision
                cubeCollis = cube.createChild('cubeCollis')
                cubeCollis.createObject('MeshObjLoader', name="loader", filename="data/mesh/smCube27.obj", triangulate="true",  scale="6")
                cubeCollis.createObject('Mesh', src="@loader")
                cubeCollis.createObject('MechanicalObject', translation='0 0 0')
                cubeCollis.createObject('Triangle')
                cubeCollis.createObject('Line')
                cubeCollis.createObject('Point')
                cubeCollis.createObject('RigidMapping')
                
                #visualization
                cubeVisu = cube.createChild('cubeVisu')
                cubeVisu.createObject('OglModel', name="Visual", fileMesh="data/mesh/smCube27.obj", color="0.0 0.1 0.5", scale="6.2")
                cubeVisu.createObject('RigidMapping')



	##########################################
        # Finger 1 Model                         #
        ##########################################
                finger1 = rootNode.createChild('finger1')
                finger1.createObject('EulerImplicit', name='odesolver', rayleighStiffness='0.1', rayleighMass='0.1')
                finger1.createObject('SparseLDLSolver', name='preconditioner')
                
                finger1.createObject('MeshVTKLoader', name='loader', filename='data/mesh/pneunetCutCoarse.vtk',translation = translateFinger1)
                finger1.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                finger1.createObject('TetrahedronSetTopologyModifier')
                finger1.createObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
                finger1.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')
                
                finger1.createObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5', rx='0', dz='0')
                finger1.createObject('UniformMass', totalMass=str(fingersMass))
                finger1.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus=str(youngModulusFingers), drawAsEdges="1")
		  
                finger1.createObject('BoxROI', name='boxROI', box='-10 0 -20 0 30 20', drawBoxes='true',doUpdate='0')
                #finger1.createObject('BoxROI', name='boxROI', box='-50 0 -20 0 30 20', drawBoxes='true',doUpdate='0')
		finger1.createObject('BoxROI', name='boxROISubTopo', box='-100 22.5 -8 -19 28 8', drawBoxes='false')
                finger1.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12', angularStiffness='1e12')
                
                finger1.createObject('LinearSolverConstraintCorrection', solverName='preconditioner')

                ##########################################
                # Sub topology                           #
                ##########################################
                modelSubTopo = finger1.createChild('modelSubTopo')
                modelSubTopo.createObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra="@boxROISubTopo.tetrahedraInROI", name='container')
                modelSubTopo.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus=str(youngModulusStiffLayerFingers-youngModulusFingers))


                ##########################################
                # Constraint                             #
                ##########################################
                cavity = finger1.createChild('cavity')
                cavity.createObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCavityCut.stl',translation = translateFinger1)
                cavity.createObject('Mesh', src='@loader', name='topo')
                cavity.createObject('MechanicalObject', name='cavity')
                cavity.createObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0.0001", triangles='@topo.triangles', visualization='0', showVisuScale='0.0002', valueType="pressure")
                cavity.createObject('BarycentricMapping', name='mapping',  mapForces='false', mapMasses='false')                
                
                ##########################################
                # Collision                              #
                ##########################################

                collisionFinger = finger1.createChild('collisionFinger')
                collisionFinger.createObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCut.stl', translation = translateFinger1)
                collisionFinger.createObject('Mesh', src='@loader', name='topo')
                collisionFinger.createObject('MechanicalObject', name='collisMech')
                collisionFinger.createObject('Triangle', selfCollision="false")
                collisionFinger.createObject('Line',selfCollision="false")
                collisionFinger.createObject('Point', selfCollision="false")
                collisionFinger.createObject('BarycentricMapping')


                ##########################################
                # Visualization                          #
                ##########################################
                modelVisu = finger1.createChild('visu')                
                modelVisu.createObject('OglModel', filename="data/mesh/pneunetCut.stl", template='ExtVec3f', color='0.7 0.7 0.7 0.6',translation = translateFinger1)
                modelVisu.createObject('BarycentricMapping')

	##########################################
        # Finger 2 Model                         #
        ##########################################
                finger2 = rootNode.createChild('finger2')
                finger2.createObject('EulerImplicit', name='odesolver', rayleighStiffness='0.1', rayleighMass='0.1')
                finger2.createObject('SparseLDLSolver', name='preconditioner')
                
                finger2.createObject('MeshVTKLoader', name='loader', filename='data/mesh/pneunetCutCoarse.vtk',rotation=str(360 - angle1*180/math.pi)+ " 0 0", translation=translateFinger2)
                finger2.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                finger2.createObject('TetrahedronSetTopologyModifier')
                finger2.createObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
                finger2.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')
                
                finger2.createObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5', rx='0', dz='0')
                finger2.createObject('UniformMass', totalMass=str(fingersMass))
                finger2.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus=str(youngModulusFingers), drawAsEdges="1")
                finger2.createObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness='1e12', angularStiffness='1e12')
                
                finger2.createObject('LinearSolverConstraintCorrection', solverName='preconditioner')

                ##########################################
                # Sub topology                           #
                ##########################################
                modelSubTopo = finger2.createChild('modelSubTopo')
                modelSubTopo.createObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra="@../../finger1/boxROISubTopo.tetrahedraInROI", name='container')
                modelSubTopo.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus=str(youngModulusStiffLayerFingers-youngModulusFingers))


                ##########################################
                # Constraint                             #
                ##########################################
                cavity = finger2.createChild('cavity')
                cavity.createObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCavityCut.stl',rotation=str(360 - angle1*180/math.pi)+ " 0 0" , translation=translateFinger2)
                cavity.createObject('Mesh', src='@loader', name='topo')
                cavity.createObject('MechanicalObject', name='cavity')
                cavity.createObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0.0001", triangles='@topo.triangles', visualization='0', showVisuScale='0.0002', valueType="pressure")
                cavity.createObject('BarycentricMapping', name='mapping',  mapForces='false', mapMasses='false')                
                
                ##########################################
                # Collision                              #
                ##########################################

                collisionFinger = finger2.createChild('collisionFinger')
                collisionFinger.createObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCut.stl',rotation=str(360 - angle1*180/math.pi)+ " 0 0", translation=translateFinger2)
                collisionFinger.createObject('Mesh', src='@loader', name='topo')
                collisionFinger.createObject('MechanicalObject', name='collisMech')
                collisionFinger.createObject('Triangle', selfCollision="false")
                collisionFinger.createObject('Line', selfCollision="false")
                collisionFinger.createObject('Point', selfCollision="false")
                collisionFinger.createObject('BarycentricMapping')


                ##########################################
                # Visualization                          #
                ##########################################
                modelVisu = finger2.createChild('visu')                
                modelVisu.createObject('OglModel', filename="data/mesh/pneunetCut.stl", template='ExtVec3f', color='0.7 0.7 0.7 0.6',rotation=str(360 - angle1*180/math.pi)+ " 0 0", translation=translateFinger2)
                modelVisu.createObject('BarycentricMapping')

                ##########################################
                # Finger 3 Model                         #
                ##########################################
                finger3 = rootNode.createChild('finger3')
                finger3.createObject('EulerImplicit', name='odesolver', rayleighStiffness='0.1', rayleighMass='0.1')
                finger3.createObject('SparseLDLSolver', name='preconditioner')
                
                finger3.createObject('MeshVTKLoader', name='loader', filename='data/mesh/pneunetCutCoarse.vtk',rotation=str(360 - angle2*180/math.pi)+ " 0 0", translation=translateFinger3)
                finger3.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                finger3.createObject('TetrahedronSetTopologyModifier')
                finger3.createObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
                finger3.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')
                
                finger3.createObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5', rx='0', dz='0')
                finger3.createObject('UniformMass', totalMass=str(fingersMass))
                finger3.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus=str(youngModulusFingers), drawAsEdges="1")
                finger3.createObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness='1e12', angularStiffness='1e12')
                
                finger3.createObject('LinearSolverConstraintCorrection', solverName='preconditioner')

                ##########################################
                # Sub topology                           #
                ##########################################
                modelSubTopo = finger3.createChild('modelSubTopo')
                modelSubTopo.createObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra="@../../finger1/boxROISubTopo.tetrahedraInROI", name='container')
                modelSubTopo.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus=str(youngModulusStiffLayerFingers-youngModulusFingers))


                ##########################################
                # Constraint                             #
                ##########################################
                cavity = finger3.createChild('cavity')
                cavity.createObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCavityCut.stl',rotation=str(360 - angle2*180/math.pi)+ " 0 0", translation=translateFinger3)
                cavity.createObject('Mesh', src='@loader', name='topo')
                cavity.createObject('MechanicalObject', name='cavity')
                cavity.createObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0.0001", triangles='@topo.triangles', visualization='0', showVisuScale='0.0002', valueType="pressure")
                cavity.createObject('BarycentricMapping', name='mapping',  mapForces='false', mapMasses='false')                

                ##########################################
                # Collision                              #
                ##########################################

                collisionFinger = finger3.createChild('collisionFinger')
                collisionFinger.createObject('MeshSTLLoader', name='loader', filename='data/mesh/pneunetCut.stl',rotation=str(360 - angle2*180/math.pi)+ " 0 0", translation=translateFinger3)
                collisionFinger.createObject('Mesh', src='@loader', name='topo')
                collisionFinger.createObject('MechanicalObject', name='collisMech')
                collisionFinger.createObject('Triangle', selfCollision="false")
                collisionFinger.createObject('Line', selfCollision="false")
                collisionFinger.createObject('Point', selfCollision="false")
                collisionFinger.createObject('BarycentricMapping')


                ##########################################
                # Visualization                          #
                ##########################################
                modelVisu = finger3.createChild('visu')                
                modelVisu.createObject('OglModel', filename="data/mesh/pneunetCut.stl", template='ExtVec3f', color='0.7 0.7 0.7 0.6',rotation=str(360 - angle2*180/math.pi)+ " 0 0", translation=translateFinger3)
                modelVisu.createObject('BarycentricMapping')

                return rootNode
```
