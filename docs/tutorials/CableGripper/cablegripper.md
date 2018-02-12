![](../../images/pluginimage.png){width=100%}

## Simulating a cable based soft gripper
This tutorial describes how to set-up a simulation environment, a scene, using Sofa and how to use SoftRobots
plug-in to model a virtual soft gripper driven by cables. The design is from [Taimoor Hassan et al.].
Once modeled in Sofa the robot can be simulated. To control our simulated robot dedicated controller can be
writen using the Python langage. In this tutorial we implements keyboard base control.

This tutorials assumes that you have sucessfully installed the ..autolink::SoftRobots plugins and
the ..autolink::STLIB plugin.


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

### Step 1: Setting up an empty scene
In this step we will explain how to setup a basic scene in Sofa.

At first, create a file named *mycablegripper.pyscn*. The content of this file is in fact standard python code that must define at least one function named
*createScene*. This function is the entry point used by Sofa to fill the simulation's content and
this is the place where you will type your scene's description.

You can try to load the totally empty scene:
<div>
```python
def createScene(rootNode):
    pass
```

<div>
<pre>
<a href="step0.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

You can change the scene content for this one and observe the result in Sofa: 
<div>
```python
from stlib.scene import MainHeader

def createScene(rootNode):
    ..autolink::STLIB::MainHeader(rootNode)
```

<div>
<pre>
<a href="step1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

You should notice that the scenegraph view have changed. This is because the ..autolink::STLIB::MainHeader creates 
a set of components that are the one needed for most common simulation scenarios. A such python function, that creates scene elements, 
will be refered as scene templates in the following. The ..autolink::STLIB::MainHeader scene template, according to its [documentation](http://stlib.readthedocs.io/en/latest/_autosummary/stlib.scene.html#stlib.scene.MainHeader), loads the following sofa components:
```qml
   {
      ..autolink::Sofa::VisualStyle,
      ..autolink::Sofa::RequiredPlugin,
      ..autolink::Sofa::OglSceneFrame,
      ..autolink::Sofa::FreeMotionAnimationLoop,
      ..autolink::Sofa::GenericConstraintSolver,
      ..autolink::Sofa::DefaultVisualManagerLoop
   }
```

On the loaded scene you can explore graphically the scene graph and the different components. By simply double clicking on the 
components you can get their internal properties and self documentations. 

### Step 2: Modeling and simulating the gripper deformations
You will now add a deformable object to your scene. There exists a lot of different mechanical behavior and
it is important to understand the one that approximates the behavior of the real object your want to simulate.
In our case, the real material is silicon which can be approximated with an elastic deformation law and
it can be simulated using the Finite Element Method. In sofa, the autolink::STLIB::ElasticMaterialObject from
*stlib.physics.deformable* provides a scenetemplate to easily add such an object in your scene.

You can then write the following scene: 
<div>
```python
from stlib.scene import ..autolink::STLIB::MainHeader
from stlib.physics.deformable import ..autolink::STLIB::ElasticMaterialObject

def createScene(rootNode):
   ..autolink::STLIB::MainHeader(rootNode)

   ..autolink::STLIB::ElasticMaterialObject(fromVolumeMesh="data/mesh/finger.vtk",
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

To compute the deformation of the object using the Finite Element Method a volumetric representation of shape
must be provided. In our case we are using a tetrahedral mesh stored in a "vtk" file. You can visualize
the computation mesh by checking the "Force Fields" option within the
*View* panel in the runSofa GUI. In case you want to learn more about volumetric mesh generation there is a dedicated
tutorial called ..autolink::SoftRobots::Docs::MeshGeneration.

Instead of displaying the tetrahedron mesh structure it is possible to wrap it into a kind of skin.
This skin is called a visual model. This visual
model being deformed according to the deformation of the tetrahedron mesh. In your scene
you can add such visual model by adding the following *withSurfaceMesh="data/mesh/finger.stl"*
to the ..autolink::STLIB::ElasticMaterialObject template's arguments.

If you click on the *Animate* button on this scene you should observe a falling object. This is because the object undergo
the gravity force but has no fixation point. To prevent the free falling, some fixation constraints are needed to a rest
position. This can be done easily by adding a box constraint in the following way:
<div>
```python
from stlib.scene import ..autolink::STLIB::MainHeader
from stlib.physics.deformable import ..autolink::STLIB::ElasticMaterialObject
from stlib.physics.constraints import ..autolink::STLIB::FixedBox as FixedBoxConstraint

def createScene(rootNode):
    ..autolink::STLIB::MainHeader(rootNode)

    finger = ..autolink::STLIB::ElasticMaterialObject(
                                   fromVolumeMesh="data/mesh/finger.vtk",
                                   withPoissonRatio=0.3,
                                   withYoungModulus=18000,
                                   withTotalMass=0.5,
                                   withSurfaceMesh="data/mesh/finger.stl",
                                   attachedTo=rootNode)

    FixedBoxConstraint(atPositions=[-15, 0, 0, 5, 10, 15],
                       applyTo=finger,
                       withVisualization=True)

    return rootNode
```

<div>
<pre>
<a href="step2.2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>


## Step 3: Actuating the finger with a cable

In the previous step, we showed how to add a finger like object made of silicone rubber.
In this step, we will explain how to actuate it using a 1d inelastic cable attached to the fingertip. 
The cable can be used to pull or release the fingertip by pressing ctrl+ and ctrl-.

To do so we needs to add the following functionalities:
 - Create and add a pulling cable and specifying its geometry
 - Use a Python script to drive the cable
 - Actuate it interactively


Use the ..autolink::SoftRobots::PullingCable template from the ..autolink::SoftRobots plugins to add
a cable.
<div>
```python
### ... similar to previous step ....
from softrobots.actuators import ..autolink::SoftRobots::PullingCable

def createScene(rootNode, name="theFinger"):
    ### ... similar to previous step ....

    cable = ..autolink::SoftRobots::PullingCable(attachedTo=finger,
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


Changing the cable lenght at run-time is possible by adding a ..autolink::Sofa::PythonScriptController. 
To do that you first need to create a python object in-heriting from *Sofa.PythonScriptController*. This
controller will be in charge of redefining the *onKeyPressed* behavior and implementing the desired one
as in the following code:
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

The controller can then be attached the objet it is supposed to control by doing:
<div>
```python
### ... similar to previous step with the added FingerController ....

def createScene(rootNode, name="theFinger"):
    ### ... similar to previous step ....

    ## This create a PythonScriptController that permits to programatically implement new behavior
    ## or interactions using the Python programming langage. 
    finger.addObject( FingerController(cable) )
```
<div>
<pre>
<a href="step3.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

If you run the resulting scene you should now be able to change the cable length resulting in
movements of the finger.

## Step 4: Defining self-collision regions
By default Sofa doesn't handle self-collisions, which in general are expensive to compute.
The drawback is that this can lead to some undesireable behaviors, i.e. self-interesections, when the object deforms a lot
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
Now we have a working single finger we can duplicate it so that it can match the orginal soft gripper
robot design.

![The soft gripper with three fingers](data/images/gripper.png){with=300}

As our *pyscn* file is getting bigger and bigger we need to refactor it to improve its modularity.
To do that we will make our own scene template by moving the finger creation into a separated file.
You can do that by moving the current code into a separated file that you will call *gripper.py*.

By default the fingers are at the same location and orientation, to change that you need
to expose the orientation, translation as well as the other location specific properties (the box fixation point)
as parameters of the *Finger* function. This should result in something like the following:
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

def Gripper(node):
    Finger(node, "Finger1",
           withRotation=[0, 0, 25], withTranslation=[150, 0, 0],
            withFixingBox=[135, 0, 0, 155, 10, 15], withPullPointLocation=[150, 12.5, 2.5])
    Finger(node, "Finger2",
           withRotation=[180, 0, -25], withTranslation=[150, 20, 0],
        withFixingBox=[135, 10, -15, 155, 30, 0], withPullPointLocation=[150, 12.5, -2.5])
    Finger(node, "Finger3",
           withRotation=[180, 0, -25], withTranslation=[150, 20, 30],
        withFixingBox=[135, 10, 15, 155, 30, 30], withPullPointLocation=[150, 12.5, 27.5])
    return node
```
</div>

Let's now make a scene template for the whole gripper by adding the following code in the *gripper.py* file:
<div>
```python
def Gripper(node):
    """Under-actuated soft gripper from [Taimoor Hassan et al.]"""
    Finger(node, "Finger1",
           withRotation=[0, 0, 25], withTranslation=[150, 0, 0],
            withFixingBox=[135, 0, 0, 155, 10, 15], withPullPointLocation=[150, 12.5, 2.5])
    Finger(node, "Finger2",
           withRotation=[180, 0, -25], withTranslation=[150, 20, 0],
        withFixingBox=[135, 10, -15, 155, 30, 0], withPullPointLocation=[150, 12.5, -2.5])
    Finger(node, "Finger3",
           withRotation=[180, 0, -25], withTranslation=[150, 20, 30],
        withFixingBox=[135, 10, 15, 155, 30, 30], withPullPointLocation=[150, 12.5, 27.5])
    return node
```
</div>

Now the *mycablegripper.pyscn* should be much simpler only containing:
<div>
```python
from stlib.scene import MainHeader, ContactHeader
from stlib.physics.collision import FrictionalContact
from stlib.physics.rigid import Floor, Cube

def createScene(rootNode):
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)
```
<pre>
<a href="step5.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>


## Step 6: Adding rigid objects to your scene

Adding as static floor is as easy as importing the scene templates from *stlib.physics.rigid* and
specify its location, size and orientation. The graspable cube is a bit more complexe as the physics
is computed at the center of gravity and an inertia matrix need to be provided.

<div>
```python
from stlib.physics.rigid import Cube, Floor
from gripper import Gripper

def createScene(rootNode):
   MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
   ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

   Gripper(attachedTo=rootNode)

   Floor(rootNode, name="Floor",
         withColor=[1.0,0.0,0.0],
         withTranslation=[0.0,-160.0,0.0],
         isAStaticObject=True)

   Cube(rootNode, name="Cube",
         withScale=20.0,
         withColor=[1.0,1.0,0.0],
         withTotalMass=0.03,
         withVolume=20,
         withInertiaMatrix=[1000.0,0.0,0.0,0.0,1000.0,0.0,0.0,0.0,1000.0],
         withTranslation=[0.0,-130.0,10.0])

   return rootNode
```

<div>
<pre>
<a href="step3.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

### Conclusion
That's all for this tutorial. You can now try to model your own soft robots and you are strongly
encouraged to pursue the other tutorial and read the thematical documentations.
