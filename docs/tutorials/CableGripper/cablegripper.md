![](../../images/pluginimage.png){width=100%}

## Simulating a cable based soft gripper
This tutorial describes how to set-up a simulation environment, a scene, using ..autolink::Sofa and how to use the
..autolink::SoftRobots plugin to model a virtual soft gripper driven by cables. The design of the gripper
we are going to model is from [Taimoor Hassan et al. ](https://dl.acm.org/citation.cfm?id=2977533).
Once modeled in Sofa the robot can be simulated and controlled.

This tutorials assumes that you have sucessfully installed ..autolink::Sofa with the ..autolink::STLIB and
..autolink::SoftRobots plugins.


<object style="width: 320px; height: 200px;" data="https://www.youtube.com/embed/9JOCe2pf34o?autoplay=0"> </object>

### Step 0: Try the simulation in Sofa
Before continuing this tutorial try the resulting simulation you need to achieved.
You can do that by clicking on the [Animate](sofa?action=animate) button in the Sofa GUI. When the
simulation is running you can manipulate the gripper by pressing the following keys:

 - ctrl + *+* to close the gripper
 - ctrl + *-* to open the gripper
 - ctrl + *arrow up* to move it up
 - ctrl + *arrow down* to move it down
 - ctrl + *arrow left* to move it left
 - ctrl + *arrow right* to move it right

Note that with MacOS, you may have to use *cmd* instead of *ctrl*.

### Step 1: Warming up by setting an empty scene
Create a file named *mycablegripper.pyscn*. The content of this file will be standard python code but you need
at least to have one function named *createScene* taking a parameter. This function is the entry point used by Sofa
to fill the simulation's content and this is the place where you will type your scene's description.

So to most simple file you can have should look like the following:
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
from stlib.scene import ..autolink::STLIB::MainHeader

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

When the scene is loaded you can explore graphically the scene graph and the different components.
By simply double clicking on the components you can get their internal properties and self documentations.

### Step 2: Modeling and simulating the gripper deformations
Add now a deformable object to your scene. There exists a lot of different mechanical behavior and
it is important to understand the one that approximates the behavior of the real object your want to simulate.
In our case, the real material is silicon which we will approximate with an elastic deformation law and
simulate using the Finite Element Method (..autolink::General::FEM). In sofa, the
..autolink::STLIB::ElasticMaterialObject from *stlib.physics.deformable* provides a scene template
to easily add such an object in your scene.

This should results in the following scene:
<div>
```python
from stlib.scene import ..autolink::STLIB::MainHeader
from stlib.physics.deformable import ..autolink::STLIB::ElasticMaterialObject

def createScene(rootNode):
   m=..autolink::STLIB::MainHeader(rootNode)
   m.getObject("..autolink::Sofa::VisualStyle").displayFlags="showForceFields"

   ..autolink::STLIB::ElasticMaterialObject(fromVolumeMesh="data/mesh/finger.vtk",
                          ..autolink::STLIB::PARAMS::withYoungModulus=18000,
                          ..autolink::STLIB::PARAMS::withPoissonRatio=0.5,
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
*View* panel in the runSofa GUI or, as we did, by changing the displayFlags property of the VisualObject.
In case you want to learn more about volumetric mesh generation there is a dedicated tutorial called
..autolink::SoftRobots::Docs::MeshGeneration.

Instead of displaying the tetrahedron mesh computational structure it is possible to wrap it into a visual model.
This visual model being deformed according to the deformation of the tetrahedron mesh. To add such
visual model you simply need to add *withSurfaceMesh="data/mesh/finger.stl"*
to the ..autolink::STLIB::ElasticMaterialObject template's arguments.

At this step if you click on the *Animate* button on this scene you should observe a falling object. This is because
the object undergo the gravity force but has no fixation point. To prevent the free falling, some constraints are
needed to a rest position. This can be done easily by adding a box constraint in the following way:
<div>
```python
from stlib.scene import ..autolink::STLIB::MainHeader
from stlib.physics.deformable import ..autolink::STLIB::ElasticMaterialObject
from stlib.physics.constraints import ..autolink::STLIB::FixedBox as FixedBoxConstraint

def createScene(rootNode):
    ..autolink::STLIB::MainHeader(rootNode)

    finger = ..autolink::STLIB::ElasticMaterialObject(
                                   fromVolumeMesh="data/mesh/finger.vtk",
                                   ..autolink::STLIB::PARAMS::withPoissonRatio=0.3,
                                   ..autolink::STLIB::PARAMS::withYoungModulus=18000,
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
We will now actuate it using a 1d inelastic cable attached to the fingertip and passing through
the object geomertry. The cable length can be increase or decrease at run-time using the
ctrl+ and ctrl-.

To model the cable you can use the ..autolink::SoftRobots::PullingCable template from
the ..autolink::SoftRobots plugin and use it in the following way:
<div>
```python
### ... similar to previous step ....
from softrobots.actuators import ..autolink::SoftRobots::PullingCable

def createScene(rootNode):
    ### ... similar to previous step ....

    ## This is here to display the cable geometry.
    m.getObject("..autolink::Sofa::VisualStyle").displayFlags="showForceFields showInteractionForceFields"


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


Changing the cable lenght at run-time is dont using a ..autolink::Sofa::PythonScriptController.
First you need to create a python object in-heriting from *Sofa.PythonScriptController*. This
object will be in charge of redefining the *onKeyPressed* behavior and implementing the desired
behavior using the following code:
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

The controller can then be attached to objet it is supposed to control by doing:
<div>
```python
### ... similar to previous step with the added FingerController ....

def createScene(rootNode):
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

If you now run the scene you should now be able to change the cable length resulting in movements
of the finger.

## Step 4: Defining self-collision regions
By default Sofa doesn't handle self-collisions because they are very expensive to compute.
The drawback is that this can lead to some undesireable behaviors, i.e. self-interesections, when the object deforms a lot
as illustrated in the figure below.

![Finger with self-intersecting](data/images/NoCollisionRegions.png){width=200}

![Without self-intersection](data/images/CollisionRegions2.png){width=200}


To activate self-collision you need to define speciallly the geometries for which collions are
checked and handled. This can be done by adding the following code to your scene.
<div>
```python
## ... Same imports as in step 3 ...
from stlib.physics.collision import ..autolink::STLIB::CollisionMesh

def createScene(rootNode):
    ..autolink::STLIB::MainHeader(rootNode)
    ..autolink::STLIB::ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)


    ## ... Remaining content of step 3 ...
    m.getObject("VisualStyle").displayFlags="showForceFields showInteractionForceFields showCollisionModels"

    ..autolink::STLIB::CollisionMesh(attachedTo=finger,
             fromSurfaceMesh="data/mesh/fingerCollision_part1.stl", withName="part1", withACollisionGroup=1)

    ..autolink::STLIB::CollisionMesh(attachedTo=finger,
              fromSurfaceMesh="data/mesh/fingerCollision_part2.stl", withName="part2", withACollisionGroup=2)
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
You can do that by moving the current code into a separated file that you will call *finger.py*.

By default the fingers are at the same location and orientation, to change that you need
to expose the orientation, translation as well as the other location specific properties (the box fixation point)
as parameters of the *Finger* function. The finger.py file should contains something similar to:
<div>
```python
import Sofa
from stlib.physics.deformable import ..autolink::STLIB::ElasticMaterialObject
from stlib.physics.constraints import ..autolink::STLIB::FixedBox as FixedBoxConstraint
from stlib.physics.collision import ..autolink::STLIB::CollisionMesh

from softrobots.actuators import ..autolink::SoftRobots::PullingCable

class FingerController(Sofa.PythonScriptController):
    def __init__(self, cable ):
        self.cableconstraintvalue = cable.getObject("CableConstraint").findData('value')
        self.name = "FingerController"

    def onKeyPressed(self,c):
        if (c == "+"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] + 1.

        elif (c == "-"):
            displacement = self.cableconstraintvalue.value[0][0] - 1.
            if(displacement < 0):
                displacement = 0
            self.cableconstraintvalue.value = displacement

def Finger(attachedTo=None, withName="Finger",
           withRotation=[0.0, 0.0, 0.0], withTranslation=[0.0, 0.0, 0.0],
           withFixingBox=[0.0,0.0,0.0], withPullPointLocation=[0.0,0.0,0.0]):

    finger = ..autolink::STLIB::ElasticMaterialObject(fromVolumeMesh="data/mesh/finger.vtk",
                                   withName=withName,
                                   withPoissonRatio=0.3,
                                   withYoungModulus=18000,
                                   withTotalMass=0.5,
                                   withSurfaceMesh="data/mesh/finger.stl",
                                   withRotation=withRotation,
                                   withTranslation=withTranslation,
                                   attachedTo=attachedTo)

    ..autolink::STLIB::FixedBoxConstraint(atPositions=withFixingBox, applyTo=finger,
                       withVisualization=True)

    cable=..autolink::SoftRobots::PullingCable(attachedTo=finger,
                 withName="PullingCable",
                 withAPullPointLocation=withPullPointLocation,
                 withRotation=withRotation,
                 withTranslation=withTranslation,
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

    ..autolink::STLIB::CollisionMesh(attachedTo=finger,
                 fromSurfaceMesh="data/mesh/finger.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withName="CollisionMesh", withACollisionGroup=[1, 2])

    ..autolink::STLIB::CollisionMesh(attachedTo=finger,
                 fromSurfaceMesh="data/mesh/fingerCollision_part1.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withName="CollisionMeshAuto1", withACollisionGroup=[1])

    ..autolink::STLIB::CollisionMesh(attachedTo=finger,
                 fromSurfaceMesh="data/mesh/fingerCollision_part2.stl",
                 withRotation=withRotation, withTranslation=withTranslation,
                 withName="CollisionMeshAuto2", withACollisionGroup=[2])

    return finger
```
</div>

At this step we can now create a file named *gripper.py* that will instanciate the three fingers and
implement a python script script controller to handle the gripper movement.
<div>
```python
import Sofa
from finger import Finger

def getTranslated(points, vec):
    r=[]
    for v in points:
        r.append( [v[0]+vec[0], v[1]+vec[1], v[2]+vec[2]] )
    return r

class GripperController(Sofa.PythonScriptController):
    def __init__(self, fingers):
        self.fingers = fingers
        self.name = "FingerController"

    def onKeyPressed(self,c):
        dir = None
        # UP key :
        if ord(c)==19:
            dir = [0.0,1.0,0.0]
        # DOWN key : rear
        elif ord(c)==21:
            dir = [0.0,-1.0,0.0]
        # LEFT key : left
        elif ord(c)==18:
            dir = [1.0,0.0,0.0]
        elif ord(c)==20:
            dir = [-1.0,0.0,0.0]

        if dir != None:
            for finger in self.fingers:
                mecaobject = finger.getObject("MechanicalObject")
                mecaobject.findData('rest_position').value = getTranslated( mecaobject.rest_position,  dir )

                cable = finger.getChild("PullingCable").getObject("CableConstraint")
                p = cable.pullPoint[0]
                cable.findData("pullPoint").value = [p[0]+dir[0], p[1]+dir[1], p[2]+dir[2]]

def Gripper(attachedTo=None):
    selfNode = attachedTo.createChild("Gripper")

    f1 = Finger(selfNode, "Finger1",
           withRotation=[0, 0, 105],
           withTranslation=[20.0,0.0, 0.0],
           withFixingBox=[-20, -10, 0, 20, 10, 15],
           withPullPointLocation=[3, 10.5, 3])

    f2 = Finger(selfNode, "Finger2",
           withRotation=[180, 0, 65],
           withTranslation=[-10.0,0.0, -4.0],
           withFixingBox=[-20, -10, -30, 20, 10, 0],
           withPullPointLocation=[3, 10.5, -7])

    f3 = Finger(selfNode, "Finger3",
           withRotation=[180, 0, 65],
           withTranslation=[-10.0,0.0, 34.0],
           withFixingBox=[-20, -10, 0, 20, 10, 50],
           withPullPointLocation=[3, 10.5, 31.5])

    selfNode.addObject(GripperController([f1,f2,f3]))

    return selfNode
```
</div>

Now the *mycablegripper.pyscn* should be much simpler only containing:
<div>
```python
from stlib.scene import ..autolink::STLIB::MainHeader, ..autolink::STLIB::ContactHeader
from gripper import Gripper

def createScene(rootNode):
    """This is my first Sofa scene"""
    ..autolink::STLIB::MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ..autolink::STLIB::ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Gripper(attachedTo=rootNode)
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
from stlib.scene import ..autolink::STLIB::MainHeader, ..autolink::STLIB::ContactHeader
from stlib.physics.rigid import ..autolink::STLIB::Floor, ..autolink::STLIB::Cube
from gripper import Gripper

def createScene(rootNode):
    """This is my first scene"""
    ..autolink::STLIB::MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ..autolink::STLIB::ContactHeader(rootNode, alarmDistance=4, contactDistance=3, withFrictionCoef=0.08)

    Gripper(attachedTo=rootNode)

    ..autolink::STLIB::Floor(rootNode, name="Floor",
          withColor=[1.0,0.0,0.0],
          withTranslation=[0.0,-160.0,0.0],
          isAStaticObject=True)

    ..autolink::STLIB::Cube(rootNode, name="Cube",
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
<a href="step6.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Run code snippet</a>
</pre>
</div>
</div>

### Conclusion
That's all for this tutorial. You can now try to model your own soft robots and you are strongly
encouraged to pursue the other tutorial and read the thematical documentations.
