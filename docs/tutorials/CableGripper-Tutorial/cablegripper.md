![](../../images/pluginimage.png){width=100%}

## Simulating a cable based soft gripper
This tutorial describes how to set-up a simulation environment, a scene, using Sofa and how to use SoftRobots
plug-in to model a virtual soft gripper driven by cables. Once modeled the robot can be simulated. To
control our simulated robot dedicated controller can be writen using the Python langage. In this tutorial
we implements keyboard base control.

### Step 0: Try the simulation in Sofa
Before continuing this tutorial it may be a good idea to try the resulting simulation you need to achieved.
You can do that by clicking on the [Animate](sofa?action=animate) button in Sofa. When the simulation is
running you can manipulate the gripper by pressing the following keys:

 - ctrl + *+* to close the gripper
 - ctrl + *-* to open the gripper
 - ctrl + *arrow up* to move it up
 - ctrl + *arrow down* to move it down
 - ctrl + *arrow left* to move it left
 - ctrl + *arrow right* to move it right

Note that with MacOS, you may have to use *cmd* instead of *ctrl*.

### Step 1: Setting up an empyt scene
In this step we will explain how to setup a basic scene in Sofa.

You first need to create a file named *mycablegripper.pyscn*.
This file should contains the following lines:

<div>
```python
def createScene(rootNode):
    rootNode.findData('gravity').value='-981.0 0 0';
    rootNode.findData('dt').value="0.01"

    rootNode.createObject('RequiredPlugin', name='SofaMiscCollision')
    rootNode.createObject('RequiredPlugin', name='SofaPython')
    rootNode.createObject('RequiredPlugin', name='SoftRobots')
    rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    ###################################################################
    # Direct simulation
    ###################################################################
    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
    rootNode.createObject('CollisionPipeline')
    rootNode.createObject('BruteForceDetection')
    rootNode.createObject('RuleBasedContactManager', name="Response",
                           response="FrictionContact", rules="0 * FrictionContact?mu=0.8" )
    rootNode.createObject('LocalMinDistance', name="Proximity",
                           alarmDistance="4", contactDistance="3", angleCone="0.01")

```

<div>
<pre>
<a href="step1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>


The content of a *pyscn* file is standard python code that must define a function called *createScene*.
This *createScene* function is mandatory as it is the one called automatically by Sofa when the file is loaded.
This function is in charge of filling the simulation's content, in this case by adding three objects
to the root node of the simulation.

The first one, *RequiredPlugin* tells Sofa that this scene required the SoftRobots plugins to run properly.

The second one adds a component that describes what data needs to be displayed using `show` / `hide` keywords. Each of the `displayFlags` is associated
to a check-box in the Visualization Panel of Sofa. In this example, we only want to display the appearance of the robot (`VisualModel`)
and the volumetric mesh that we will generate (`BehaviorModel`). The `BehaviorModel` is only used for debug purpose and will not be displayed
in the final version of the simulation.

The third one adds a component that describes specify the background color or image of the simulation of the 3D view.

The fourth one adds a rigid frame system in the 3D view. This one is very useful to help the developer understand how the different
objects are located in the 3D space.

### Step 2: Modeling and simulating the gripper deformations
The second step of this tutorial is to allow the finger to be mechanically simulated. This implies to:

 - Provide a solving method
 - volumetric mesh and mass properties to compute deformations and energy
 - mechanical model of the deformation
 - An attach point

Additionnaly, a visual model has been added, with a mapping between the tetrahedron model and the stl-geometry to provide
visualization of the deformations produced

<div>
```python
import os
path = os.path.dirname(os.path.abspath(__file__))+'/data/mesh/'

def createAndAddToNode(rootNode, name="theFinger"):
    finger = rootNode.createChild(name)
    finger.createObject('EulerImplicit', name='odesolver', firstOrder='1')
    finger.createObject('SparseLDLSolver', name='preconditioner')

    finger.createObject('MeshVTKLoader', name='loader', filename=path+'finger.vtk')
    finger.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    finger.createObject('MechanicalObject', name='tetras', template='Vec3d')

    ## To be properly simulated and to interact with gravity or inertia forces, an object
    ## also needs a mass. You can add a given mass with a uniform distribution for an object
    ## by adding a UniformMass component to the finger node
    finger.createObject('UniformMass', totalmass=' 0.5')

    ## The next component to add is a FEM forcefield which defines how the object reacts
    ## to a loading (i.e. which deformations are created from forces applied onto it).
    ## Here, because the finger is made of silicone, its mechanical behavior is assumed elastic.
    ## This behavior is available via the TetrahedronFEMForceField component.
    finger.createObject('TetrahedronFEMForceField', template='Vec3d',
                        name='FEM', method='large', poissonRatio='0.3',  youngModulus='18000')

    ## Set the ROI of points of the model to fix.
    ## You can either use "BoxROI"...
    finger.createObject('BoxROI', name='ROI1', box='-15 0 0 5 10 15', drawBoxes='true')
    finger.createObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness='1e12')

    finger.createObject('LinearSolverConstraintCorrection')

    #################################################################################
    ## Visualization
    fingerVisu = finger.createChild('visual')

    ## Add to this empty node a rendering model made of triangles and loaded from an stl file.
    fingerVisu.createObject('OglModel', filename=path+"finger.stl",
                            template='ExtVec3f', color="0.0 0.7 0.7")

    ## Add a BarycentricMapping to deform the rendering model to follow the ones of the
    ## mechanical model.
    fingerVisu.createObject('BarycentricMapping')

def createScene(rootNode):
    ### ... similar to step 1 ....

    createAndAddToNode(rootNode)
```

<div>
<pre>
<a href="step2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

## Step 3: Actuating the finger with a cable

In the previous step, we showed how to model and simulate a soft robot with a finger shape and made of a deformable material (silicone rubber). In this step, we will explain how to actuate it using a 1d inelastic cable attached to the fingertip. The cable can be used to pull or release the fingertip by pressing ctrl+ and ctrl-.

This scene adds the following functionalities:
 - Create a cable actuator based in constraints
 - Include a mechanical mapping to link the cable motion to the object deformation
 - Use a Python script to drive the cable
 - Actuate it interactively

<div>
```python
def createAndAddToNode(rootNode, name="theFinger"):
    ### ... similar to step 2 ....

    #################################################################################
    ## Cable
    cable = finger.createChild('cable')
    cable.createObject('MechanicalObject',
    position=(
        " -17.5 12.5 2.5 " +
        " -32.5 12.5 2.5 " +
        " -47.5 12.5 2.5 " +
        " -62.5 12.5 2.5 " +
        " -77.5 12.5 2.5 " +
        " -83.5 12.5 4.5 " +
        " -85.5 12.5 6.5 " +
        " -85.5 12.5 8.5 " +
        " -83.5 12.5 10.5 " +
        " -77.5 12.5 12.5 " +
        " -62.5 12.5 12.5 " +
        " -47.5 12.5 12.5 " +
        " -32.5 12.5 12.5 " +
        " -17.5 12.5 12.5 " ))

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last indice is where the pullPoint is connected.
    cable.createObject('CableConstraint', name="aCable",
                        indices='0 1 2 3 4 5 6 7 8 9 10 11 12 13',
                        pullPoint="0.0 12.5 2.5")

    # This create a BarycentricMapping. A BarycentricMapping is a key element as it will
    # create a bi-directional link between the cable's DoFs and the finger's ones so that movements
    # of the cable's DoFs will be mapped
    # to the finger and vice-versa;
    cable.createObject('BarycentricMapping')
```
</div>


Changing the cable lenght at run-time is possible by adding a *PythonScriptController*. To do that
you first need to add a new file called *fingercontroller.py* with the following content:
<div>
```python
# -*- coding: utf-8 -*-
import Sofa

class FingerCableController(Sofa.PythonScriptController):
    def initGraph(self, node):
        self.node = node

    def onKeyPressed(self,c):
        inputvalue = self.node.getObject('aCable').findData('value')

        if (c == "+"):
            inputvalue.value =  inputvalue.value[0][0] + 1.

        elif (c == "-"):
            displacement = inputvalue.value[0][0] - 1.
            if(displacement < 0):
                displacement = 0
            inputvalue.value = displacement
```
</div>

Each controller must inherit from the Sofa.PythonScriptController and redefines some method.
In our case we will redefine the *onKeyPressed* to implement the desired behavior.

<div>
```python
def createAndAddToNode(rootNode, name="theFinger"):
    ## ...

    ## This create a PythonScriptController that permits to programatically implement new behavior
    ## or interactions using the Python programming langage. The controller is referring to a
    ## file named "controller.py".
    cable.createObject('PythonScriptController',
                        filename="fingercontroller.py", classname="FingerCableController",
                        autoreload="true")

```
<div>
<pre>
<a href="step3.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>


## Step 4: Defining self-collision regions
By default Sofa doesn't handle self-collisions, which in general are expensive to compute.
This can lead to some undesireable results, i.e. self-interesections, when the object deforms a lot
as illustrated in the figure below.

![Finger with self-intersecting](data/images/NoCollisionRegions.png){width=200}

![Without self-intersection](data/images/CollisionRegions2.png){width=200}


To prevent these self-interesections it is possible to define special geometries for which collions are
checked and handled. In the following figure, the collision regions are shown in orange.

<div>
```python
def createAndAddToNode(rootNode, name="theFinger"):
    ## ... Content of step 3 ...

    #################################################################################
    ## Contact
    ## Add a collision model
    contactPart1 = finger.createChild('contactPart1')

    ## 1- Load the surface mesh for the collision
    contactPart1.createObject('MeshSTLLoader', name="loader", filename=path+"fingerCollision_part1.stl")
    contactPart1.createObject('Mesh', src="@loader")
    contactPart1.createObject('MechanicalObject')

    # 2- Add a collision model. These three components (Point, Line, Triangle) have to be used together.
    #    Other collision model exist (for example SphereModel)
    #    Collision model of the same group won't collide.
    contactPart1.createObject('PointModel', group="1")
    contactPart1.createObject('LineModel', group="1")
    contactPart1.createObject('TriangleModel', group="1")

    # 3- Add a mapping to link the collision model to the mechanics
    contactPart1.createObject('BarycentricMapping', mapForces="false", mapMasses="false")

    contactPart2 = finger.createChild('contactPart2')
    contactPart2.createObject('MeshSTLLoader', name="loader", filename=path+"fingerCollision_part2.stl")
    contactPart2.createObject('Mesh', src="@loader")
    contactPart2.createObject('MechanicalObject')
    contactPart2.createObject('Point', group="2")
    contactPart2.createObject('Line', group="2")
    contactPart2.createObject('Triangle', group="2")
    contactPart2.createObject('BarycentricMapping', mapForces="false", mapMasses="false")
```
<div>
<pre>
<a href="step4.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

## Step 5: Duplicating the finger to make a complete gripper
In the previous steps, we showed the simulation of a single finger made of elastic material.
But the orginal soft gripper robot is composed of three fingers actuated with cables.
In the design of Taimoor Hassan et al., a single cable is used for the three fingers.

![The soft gripper with three fingers](data/images/gripper.png){with=300}

Our pyscn file is getting bigger and bigger and is not really modular. Let's improve the situation
by moving the finger creation into a separated file to favor reusability. You can do that by cutting & pasting
the function *createAndAddToNode* as well as the two first lines into a new file called *finger.py*.

Once this is done it is now possible to use & re-use the finger in any scene in the following way:
<div>
```python
import finger

def createScene(rootNode):
    ### ... similar to previously but with the following at the end...

    finger.createAndAddToNode(rootNode, "finger1")
    finger.createAndAddToNode(rootNode, "finger2")
    finger.createAndAddToNode(rootNode, "finger3")

    return rootNode
```
<pre>
<a href="step5.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

By default the three fingers are at the same location and orientation, to change that you need
to make the orientation, translation, scale, and other location specific properties parameters of the
*createAndAddToNode* function. Here is how should look like the parameterized version of the finger.py.

<div>
```python
def createAndAddToNode(rootNode, name="theFinger",
                       rotation="0 0 0", translation="0 0 0",
                       fixingbox='0 0 0 0 0 0', pullpoint="0 0 0"):
    #....
    finger.createObject('MeshVTKLoader', name='loader', filename=path+'finger.vtk'
                        ,rotation=rotation, translation=translation)

    #...
    finger.createObject('BoxROI', name='ROI1', box=fixingbox, drawBoxes='true')

    #...
    fingerVisu.createObject('OglModel', filename=path+"finger.stl",
                            template='ExtVec3f', color="0.0 0.7 0.7",
                            rotation=rotation, translation=translation
                            )

    #...
    cable.createObject('MechanicalObject', rotation=rotation, translation=translation,
    # position=(

    #...
    cable.createObject('CableConstraint', name="aCable",
                       indices='0 1 2 3 4 5 6 7 8 9 10 11 12 13',
                       pullPoint=pullpoints)

    #...
    contactPart1.createObject('MeshSTLLoader', name="loader",
                              filename=path+"fingerCollision_part1.stl",
                              rotation=rotation, translation=translation)

    #...
    contactPart2.createObject('MeshSTLLoader', name="loader",
                              filename=path+"fingerCollision_part2.stl",
                              rotation=rotation, translation=translation)

```
</div>

And in the *mycablegripper.pyscn* file specify the translation/rotation, fixing box & pulling
point in the following way:
<div>
```python
import finger

def createScene(rootNode):
    ### ... similar to previously but with the following at the end...

    finger.createAndAddToNode(rootNode, "finger1",
                           rotation="0 0 25", translation="150 0 0",
                           fixingbox='135 0 0 155 10 15', pullpoint="150 12.5 2.5")
    finger.createAndAddToNode(rootNode, "finger2",
                           rotation="180 0 -25", translation="150 20 0",
                           fixingbox='135 10 -15 155 30 0', pullpoint="150 12.5 -2.5")
    finger.createAndAddToNode(rootNode, "finger3",
                           rotation="180 0 -25", translation="150 20 30",
                           fixingbox='135 10 15 155 30 30', pullpoint="150 12.5 27.5")


    return rootNode
```
<pre>
<a href="step5.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

There is still some work to do in order to correctly provide the orientation, and location
We simulate that by applying the same control input to three cable actuators placed on the three fingers.

In the file scene, we put 3 finger models (instead of 1) by copy/paste of the previous simulations.
A 'PythonScriptController' is placed at the root node of the graph to interactively change the inputs of
3 the 'CableConstraint'  components. Here, we use a control of the actuator, i.e. direct or fowrward simulation.



## Step 6: Adding the ground plane

In the same directory, create a new file named *floor.py* with the following content:
<div>
```python
def createAndAddToNode(rootNode, name="thePlane",
                       rotation="0 0 270", translation="38 0 0", scale=10):

    planeNode = rootNode.createChild(name)
    planeNode.createObject('MeshObjLoader', name='loader',
                            filename="mesh/floorFlat.obj", triangulate="true",
                            rotation=rotation, scale=scale, translation=translation)

    planeNode.createObject('Mesh', src="@loader")
    planeNode.createObject('MechanicalObject', src="@loader")
    planeNode.createObject('Triangle',simulated="0", moving="0")
    planeNode.createObject('Line',simulated="0", moving="0")
    planeNode.createObject('Point',simulated="0", moving="0")
    planeNode.createObject('OglModel',name="Visual", fileMesh="mesh/floorFlat.obj",
                            color="1 0 0 1",
                            rotation=rotation, scale=scale, translation=translation)

    return planeNode
```
</div>

You can then add the floor by changing to the *mycablegripper.pyscn* file with the following content:
<div>
```python
import floor

def createScene(rootNode):
   ## .... The content of the previous step ...

   floor.createAndAddToNode(rootNode)
```

<div>
<pre>
<a href="step2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>



## Step 7: Adding the manipulated object
In the same directory, create a new file named *cube.py* with the following content:
<div>
```python
def createAndAddToNode(rootNode, name="theCube"):
	################################ Grasped Object ###################################
	# mechanics
	cube =rootNode.createChild(name)
	cube.createObject('EulerImplicit', name='odesolver')
	cube.createObject('CGLinearSolver', name='linearSolver')
	cube.createObject('MechanicalObject', template="Rigid", scale="6", dx="67.0", dy="10", dz="8", rx="10" ,ry="10")
	cube.createObject('UniformMass', mass='0.03 10 1000 0 0 0 1000 0 0 0 1000')
	cube.createObject('UncoupledConstraintCorrection')

	#collision
	cubeCollis = cube.createChild('cubeCollis')
	cubeCollis.createObject('MeshObjLoader', name="loader", filename="mesh/smCube27.obj", triangulate="true",  scale="6")
	cubeCollis.createObject('Mesh', src="@loader")
	cubeCollis.createObject('MechanicalObject')
	cubeCollis.createObject('Triangle')
	cubeCollis.createObject('Line')
	cubeCollis.createObject('Point')
	cubeCollis.createObject('RigidMapping')

	#visualization
	cubeVisu = cube.createChild('cubeVisu')
	cubeVisu.createObject('OglModel', name="Visual", fileMesh="mesh/smCube27.obj", color="0.0 0.1 0.5", scale="6.2")
	cubeVisu.createObject('RigidMapping')

        return cube
```
</div>

You can then add the floor by changing to the *mycablegripper.pyscn* file with the following content:
<div>
```python

import floor
import cube

def createScene(rootNode):
   ## .... The content of the previous step ...

   cube.createAndAddToNode(rootNode)
```

<div>
<pre>
<a href="step3.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>


### How to use the demo

After clicking on "Animate" you can click in the 3d view then pulling/releasing the three cables by pressing CTRL+  and CTLR-.

![the gripper actuated](data/images/gripper2.png)

## Step 5: Grasping simulation

[Open Step 6 in Sofa](06-GraspingSimulation.pyscn)


The simulation in SOFA allows to reproduce grasping using a combination of constraints for frictional contact (using Coulomb's friction) and for actuators. To do this, we need to modify several elements of the simulation.

### A. Activate collision and direct simulation

In this exemple, we will simulate the collision response of the gripper with its environment and particularly an object that will be grasped. The first step is to activate the modules for collision detection and response in SOFA. This modules are described in more details in the documentation of SOFA. The user can modify the coefficient of friction in RuleBasedContactManager (mu=0.8). GenericConstraintSolver is able to solve, in a unified system, the constraints         for collision response and the constraints (direct, non inverse) of the actuators.

### B. Add an object that will be grasped

![description](data/images/image1.png)

We add a rigid object, and simulate its dynamics with a 6Dof mechanical object. It computes the physics at the center of gravity. The mass and inertia of the object are computed by UniformMass. The component UncoupledConstraintCorrection provides the physics-based response after collision (in practice, it will use the parameters of the component UniformMass).
Then, we will need to add a collision model to detect the contact between this cube and the soft gripper. First, we load the mesh that is used for collision and create a mechanical object with it. Then, we add the collision primitive (Triangle, Line and Point) that will be tested. Finally, the collision model is linked to the behavior of the center of gravity by a RigidMapping.

![descr](data/images/image11.png)

In the same way, we can load an other mesh and use a mapping for the visualization.

### C. Add a fixed floor

Now, we add a collision surface collision plane so that we can place the object before and after grasping:

![descr](data/images/image2.png)

### D. Add collision models to the gripper


We need to put a collision surface to each finger of the soft gripper. In the graph, the collision model is a child of the node that contains the mechanical object (here, the FEM model of each finger). This surface is defined by a mesh (here finger.stl). The primitive of collision are also defined (Triangle, Line, Point). The surface motion is linked to the motion of the FEM model using a BarycentricMapping:

![descr](data/images/image21.png)

### E. Control the gripper

The user can interactively control the gripper using keyboard events. The definition of the controls is done using a python script. To close/open the gripper, press the following keys:

 - ctrl + '+'
 - ctrl + '-'

Note that with MacOS, you may have to use *cmd* instead of *ctrl*. The actuator are controlled in force. They key ctrl + increased the force applied on the cable. To move the gripper, press the following keys:


### Volumetric Mesh Generation

Volumetric mesh generation is a complex topic and several tools exists to perform this task such as: [GID](http://www.gidhome.com/), [CGAL](http://www.cgal.org/) or [Gmsh](http://geuz.org/gmsh/). We choose to use the C++ library `CGAL` ([CGAL](http://www.cgal.org)) as a dedicated plug-in for Sofa exists. Volumetric mesh can be computed either using a surface mesh or an image. Whatever the input, we need to add the `CGALPlugin` with the following line:

~~~
rootNode.createObject('RequiredPlugin', pluginName='CGALPlugin')
~~~

All the components that will help to build the volumetric mesh will be placed in a child node, this is performed with

~~~
node = rootNode.createChild('node')
~~~

Then, we need to load the surface mesh that will be used as input for CGAL. Many file formats are supported (`OBJ`, `VTK`, `STL`...). In our example, a STL file has been produced using some CAD software. This file is then loaded with:

~~~
node.createObject('MeshSTLLoader', name='mesh', filename=path+'finger.stl')
~~~

And then the magic is performed with the `MeshGenerationFromPolyhedron` where four parameters are used to control the meshing:

 - `cellSize`: this parameter controls the size of mesh tetrahedra. It provides an upper bound on the circumradius of the mesh tetrahedra
 - `facetAngle`: This parameter controls the shape of surface facets. Actually, it is a lower bound for the angle (in degree) of surface facets. When boundary surfaces are smooth, the termination of the meshing process is guaranteed if the angular bound is at most 30 degrees
 - `facetAngle`: this parameter controls the shape of surface facets. Actually, it is a lower bound for the angle (in degree) of surface facets. When boundary surfaces are smooth, the termination of the meshing process is guaranteed if the angular bound is at most 30 degrees
 - `cellRatio`: this parameter controls the shape of mesh cells. Actually, it is an upper bound for the ratio between the circumradius of a mesh tetrahedron and its shortest edge. There is a theoretical bound for this parameter: the Delaunay refinement process is guaranteed to terminate for values larger than 2
 - `facetApproximation`: the approximation error between the boundary and the subdivision surface. It provides an upper bound for the distance between the circumcenter of a surface facet and the center of a surface Delaunay ball of this facet

It may require some trials and errors to find a good set of parameters that capture well the details of the surface mesh without leading to a large number of tetrahedra. The framerate of the simulation is quite sensitive to the setting of these parameters. If the simulation is running to slow consider changing them in order to reduce the number of tetrahedra. For our example, the set of parameters is:

~~~
node.createObject('MeshGenerationFromPolyhedron', name='gen', template='Vec3d', inputPoints='@mesh.position', inputTriangles='@mesh.triangles', drawTetras='1',
    cellSize="10",
    facetAngle="30",
    facetSize="4",
    cellRatio="2",   #Convergence problem if lower than 2
    facetApproximation="1"
    )
~~~

The computed tetrahedra are then stored in a mesh container for later usage with:

~~~
node.createObject('Mesh', position='@gen.outputPoints', tetrahedra='@gen.outputTetras')
~~~

Mind the fact that the syntax used links the output of the generator 'gen' to the created 'Mesh' object using '@'. After that you may export the resulting volumetric mesh with the following line for further use:

~~~
node.createObject('VTKExporter', filename=path+'finger', edges='0', tetras='1', exportAtBegin='1')
~~~

We want to export only the tetrahedra (no edges, no triangles) and we want a single export that is performed at the beginning of the simulation (a single export is needed since the mesh will not deform during this simulation).

For an interactive feedback of what has been computed by `CGAL`, we can use this line:

~~~
node.createObject('OglModel', filename=path+"finger.stl", color="0.0 0.7 0.7 0.5")
~~~

It will superimpose the surface mesh on the volumetric mesh.



