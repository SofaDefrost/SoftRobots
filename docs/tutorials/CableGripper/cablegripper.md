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
from stlib.scene import MainHeader

def createScene(rootNode):
    MainHeader(rootNode)
```

<div>
<pre>
<a href="step1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

The content of a *pyscn* file is standard python code that must define a function called *createScene*.
This *createScene* function is mandatory and is called automatically by Sofa when the file is loaded.
This function is in charge of filling the simulation's content and this is where you type your scene's
description.

The *MainHeader* line is a scene templates from a library
called STLIB. [View STLIB Doc](http://stlib.readthedocs.io/en/latest/)

This template adds set of selected components that are needed for most of simulation. Once loaded you
can explore the loaded scene graph in the sofa GUI and, by double clicking on components, get
their internal properties and self documentations.

### Step 2: Modeling and simulating the gripper deformations
The second step of this tutorial is add a deformable object that will be mechanically simulated.
As the real material we want to simulate is made of silicons it can be approximated with and elastic
deformation law. The *ElasticMaterialObject* from *stlib.physics.deformable* provide a scene template
for such an objet.

<div>
```python
from stlib.scene import MainHeader
from stlib.physics.deformable ElasticMaterialObject

def createScene(rootNode):
    MainHeader(rootNode)

    ElasticMaterialObject(fromVolumeMesh="data/mesh/finger.vtk",
                          withYoungModulus=18000,
                          withPoissonRatio=0.5,
                          withTotalMass=0.5,
                          attachedTo=rootNode)
```

<div>
<pre>
<a href="step2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

Running the previously defined scene results in a infinitly falling finger object. This is because
the scene is missing the definition of a constraint that will "attached" the object to a given
position. This can be done easily by adding a box


<div>
```python
from stlib.scene import MainHeader
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.constraints import FixedBox as FixedBoxConstraint

def createScene(rootNode):
    MainHeader(rootNode)

    finger = ElasticMaterialObject(fromVolumeMesh="data/mesh/finger.vtk",
                                   withPoissonRatio=0.3,
                                   withYoungModulus=18000,
                                   withTotalMass=0.5,
                                   attachedTo=rootNode)

    FixedBoxConstraint(atPositions=[-15, 0, 0, 5, 10, 15], applyTo=finger,
                       withVisualization=True)

    return rootNode
```

<div>
<pre>
<a href="step3.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

Finally, instead of displaying the tetrahedron mesh structure use for the deformation computation
it is possible to wrap it into a kind of skin. This skin is called a visual model. This visual
model being deformed according to the deformation of the tetrahedron mesh. In your scene
you can add such visual model by adding the following *withSurfaceMesh="data/mesh/finger.stl"*
to the ElasticMaterialObject template's arguments.


## Step 3: Actuating the finger with a cable

In the previous step, we showed how to model and simulate a soft robot with a finger shape
made of a deformable material (silicone rubber). In this step, we will explain how to actuate
it using a 1d inelastic cable attached to the fingertip. The cable can be used to pull or
release the fingertip by pressing ctrl+ and ctrl-.

This means adding the following functionalities:
 - Create and add a pulling cable with a given geometry
 - Use a Python script to drive the cable
 - Actuate it interactively

<div>
```python
def createScene(rootNode, name="theFinger"):
    ### ... similar to previous step ....

    PullingCable(attachedTo=finger,
             withAPullPointLocation=[0.0, 12.5, 2.5],
             withCableGeometry=[[-17.5, 12.5, 2.5],
                                [-32.5, 12.5, 2.5],
                                [-47.5, 12.5, 2.5],
                                [-62.5, 12.5, 2.5],
                                [-77.5, 12.5, 2.5],
                                [-83.5, 12.5, 4.5],
                                [-85.5, 12.5, 6.5],
                                [-85.5, 12.5, 8.5],
                                [-83.5, 12.5, 10.5],
                                [-77.5, 12.5, 12.5],
                                [-62.5, 12.5, 12.5],
                                [-47.5, 12.5, 12.5],
                                [-32.5, 12.5, 12.5],
                                [-17.5, 12.5, 12.5]]);
```
</div>


Changing the cable lenght at run-time is possible by adding a *PythonScriptController*. To do that
you first need to create a python object in-heriting from *Sofa.PythonScriptController*. The
controller will be in charge of redefine the *onKeyPressed* to implement the desired behavior.
This can be done by adding the following at the beginning of your scene.
<div>
```python
import Sofa
class FingerController(Sofa.PythonScriptController):
    def __init__(self, cable):
        self.cableconstraintvalue = cable.getObject("CableConstraint").findData('value')

    def onKeyPressed(self,c):
        if (c == "+"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] + 1.

        elif (c == "-"):
            displacement = self.cableconstraintvalue.value[0][0] - 1.
            if(displacement < 0):
                displacement = 0
            self.cableconstraintvalue.value = displacement
```
</div>

The created controller can then be attached the objet it is supposed to control in the following
way:

<div>
```python
def createScene(rootNode, name="theFinger"):
    ### ... similar to previous step ....

    ## This create a PythonScriptController that permits to programatically implement new behavior
    ## or interactions using the Python programming langage. The controller is referring to a
    ## file named "controller.py".
    finger.addObject( FingerController(cable) )
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
from stlib.physics.collision import CollisionMesh
from stlib.physics.collision import FrictionalContact

def createScene(rootNode):
    ## ... Content of step 3 ...

    CollisionMesh(attachedTo=finger,
             fromSurfaceMesh="data/mesh/fingerCollision_part1.stl", withName="part1", withACollisionGroup=1)
    CollisionMesh(attachedTo=finger,
              fromSurfaceMesh="data/mesh/fingerCollision_part2.stl", withName="part2", withACollisionGroup=2)

    FrictionalContact(applyTo=finger)
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
by moving the finger creation into a separated file to favor modularity and reusability. You can do
that by moving the current code into a separated file called *finger.py*.

Once this is done it is now possible to use & re-use the finger in any scene in the following way:
<div>
```python
from stlib.scene import MainHeader
from stlib.physics.collision import FrictionalContact
from finger import Finger

def createScene(rootNode):
    MainHeader(rootNode, plugins=["SoftRobots"])
    FrictionalContact(applyTo=rootNode)

    Finger(attachedTo=rootNode, withName="Finger1")
    Finger(attachedTo=rootNode, withName="Finger2")
    Finger(attachedTo=rootNode, withName="Finger3")

    return rootNode
```
<pre>
<a href="step5.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

By default the three fingers are at the same location and orientation, to change that you need
to make the orientation, translation and other location specific properties as template parameters
for the *Finger* function. Here is how should look like the parameterized version of the finger.py.

<div>
```python
def Finger(attachedTo=None, withName="Finger",
           withRotation=[0.0, 0.0, 0.0], withTranslation=[0.0, 0.0, 0.0],
           withFixingBox=[0.0,0.0,0.0], withPullPointLocation=[0.0,0.0,0.0]):

    finger = ElasticMaterialObject(fromVolumeMesh="data/mesh/finger.vtk",
                                   withName=withName,
                                   withPoissonRatio=0.3,
                                   withYoungModulus=18000,
                                   withTotalMass=0.5,
                                   withSurfaceMesh="data/mesh/finger.stl",
                                   withRotation=withRotation,
                                   withTranslation=withTranslation,
                                   attachedTo=attachedTo)

    FixedBoxConstraint(atPositions=withFixingBox, applyTo=finger,
                       withVisualization=True)

    cable=PullingCable(attachedTo=finger,
                 withName="cable",
                 withAPullPointLocation=withPullPointLocation,
                 withCableGeometry=[[-17.5, 12.5, 2.5],
                                    [-32.5, 12.5, 2.5],
                                    [-47.5, 12.5, 2.5],
                                    [-62.5, 12.5, 2.5],
                                    [-77.5, 12.5, 2.5],
                                    [-83.5, 12.5, 4.5],
                                    [-85.5, 12.5, 6.5],
                                    [-85.5, 12.5, 8.5],
                                    [-83.5, 12.5, 10.5],
                                    [-77.5, 12.5, 12.5],
                                    [-62.5, 12.5, 12.5],
                                    [-47.5, 12.5, 12.5],
                                    [-32.5, 12.5, 12.5],
                                    [-17.5, 12.5, 12.5]]);

    finger.addObject( FingerController(cable) )

    CollisionMesh(attachedTo=finger,
                 fromSurfaceMesh="data/mesh/fingerCollision_part1.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withName="part1", withACollisionGroup=1)
    CollisionMesh(attachedTo=finger,
                  fromSurfaceMesh="data/mesh/fingerCollision_part2.stl",
                  withRotation=withRotation, withTranslation=withTranslation,
                  withName="part2", withACollisionGroup=2)

    return finger
```
</div>

And in the *mycablegripper.pyscn* file specify the translation/rotation, fixing box & pulling
point in the following way:
<div>
```python
import finger

def createScene(rootNode):
    ### ... similar to previously but with the following at the end...

    Finger(rootNode, "Finger1",
           withRotation=[0, 0, 25], withTranslation=[150, 0, 0],
           withFixingBox=[135, 0, 0, 155, 10, 15], withPullPointLocation=[150, 12.5, 2.5])
    Finger(rootNode, "Finger2",
           withRotation=[180, 0, -25], withTranslation=[150, 20, 0],
           withFixingBox=[135, 10, -15, 155, 30, 0], withPullPointLocation=[150, 12.5, -2.5])
    Finger(rootNode, "Finger3",
           withRotation=[180, 0, -25], withTranslation=[150, 20, 30],
           withFixingBox=[135, 10, 15, 155, 30, 30], withPullPointLocation=[150, 12.5, 27.5])

    return rootNode
```
<pre>
<a href="step5.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

That's all for the gripper.

## Step 6: Adding rigid objects to your scene

Adding the floor & cube is as easy as importing the scene templates from *stlib.physics.rigid* :
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



