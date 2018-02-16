![](../../images/pluginimage.png){width=100%}

## Simulating a cable based soft gripper
This tutorial describes how to set-up a simulation environment, a scene, using ..autolink::Sofa and how to use the
..autolink::SoftRobots plugin to model a virtual soft gripper driven by cables. The design of the gripper
we are going to model is from [Taimoor Hassan et al. ](https://dl.acm.org/citation.cfm?id=2977533).
Once modeled in Sofa the robot can be simulated and controlled.

This tutorials prequisites:

- that you have sucessfully installed ..autolink::Sofa with the ..autolink::STLIB and
..autolink::SoftRobots plugins

- have basic knowledge of the ..autolink::General::Python programming language. If this is not
the case we strongly advise you to first complete a python tutorial.

- have basic knowledge of making scene with Sofa. It may be a good idea to complete the ..autlink::SoftRobots::Tutorials::RealNovice first


<center><object style="width: 320px; height: 200px;" data="https://www.youtube.com/embed/9JOCe2pf34o?autoplay=0"> </object></center>

### Step 0: Try the simulation in Sofa
Before continuing this tutorial try the resulting simulation you need to achieved.
You can do that by clicking on the [Animate] button in the Sofa GUI. When the
simulation is running you can manipulate the gripper by pressing the following keys:

 - ctrl + *+* to close the gripper
 - ctrl + *-* to open the gripper
 - ctrl + *arrow up* to move it up
 - ctrl + *arrow down* to move it down
 - ctrl + *arrow left* to move it left
 - ctrl + *arrow right* to move it right

Note that with MacOS, you may have to use *cmd* instead of *ctrl*.

<div>
<pre>
<a href="details/step0.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try now the scene in Sofa.</a>
</pre>
</div>

### Step 1: Warming up by setting an simple scene
Sofa is loading the description of the simulation from *pyscn* files. The content of these file is in fact standard python code with 
at least one function named *createScene* taking a single parameter, the root of the scene hierarchy. This function is the entry point used by Sofa
to fill the simulation's content and this is the place where you will type your scene's description. A scene is an ordered tree of nodes (ex:gripper.), with parent/child relationship (ex: finger). Each node has one or a few components. Every node and component has a name and a few features. The main node at the top of the tree is called "rootNode".

A very simple scene may look like:
<div>
```python
from stlib.scene import ..autolink::STLIB::MainHeader, ..autolink::STLIB::ContactHeader
from stlib.physics.rigid import ..autolink::STLIB::Floor
from stlib.visuals import ShowGrid

def createScene(rootNode):
    """This is my first scene"""
    m=..autolink::STLIB::MainHeader(rootNode, plugins=["SoftRobots"]) 
    ShowGrid(rootNode) 

    ..autolink::STLIB::Floor(rootNode, name="Floor",
          withTranslation=[0.0,-160.0,0.0],
          isAStaticObject=True)
    
    return rootNode
```

<div>
<pre>
<a href="details/step1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene.</a>
</pre>
</div>
</div>

Both ..autolink::STLIB::MainHeader and ..autolink::STLIB::Floor are functions that creates set of Sofa components. Such functions, that creates scene elements, will be refered as scene templates in the following of this tutorial. As an example the ..autolink::STLIB::MainHeader template, according to its [documentation](http://stlib.readthedocs.io/en/latest/_autosummary/stlib.scene.html#stlib.scene.MainHeader), loads the following sofa components into the scene graph (the hierarchical structure you can see in the left side of the Sofa GUI):
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


Here is a set of tasks you can do to get better understanding of this step.
 
####<i>Task 1.0:</i>
- Download and extract the file myproject.zip to any location of your choice. 
- Try the scene by doing runSofa cablegripper.pyscn -i. With the '-i' option Sofa will reload the scene eache time it is modified & saved. 
- Explore the scene graph and the properties of the different objects it contains in the Sofa GUI. 

####<i>Task 1.1:</i>
..autolink::FIXME...

####<i>Task 1.2:</i>
- Add a rigid Cube. The cube must be of size '20'.

####<i>Task 1.3:</i>
- Add a serie of 5 cubes in a row with equidistant spacing and different color. 
(Note, with Sofa colors are encoded as a triplet of float encoding bewteen 0.0 and 1.0 the intensity of the Red/Green/Blue component. a ..autolink::STLIB::Floor is a specific kind of ..autolink::STLIB::RigidObject thus it have the same parameters).

####<i>Task 1.4:</i>
- Add the Gripper object to the scene in a way similar to the Floor the Cubes. Normally, only a cylindric shape should 
appear indicating the location of the Gripper base. 

### Step 2: Modeling and simulating the gripper deformations
We will now add a deformable object to a scene. There exists a lot of different mechanical behavior and
it is important to understand the one that approximates the behavior of the real object your want to simulate.
In particular, it is important to know how soft or stiff the material is, if it has an elastic or more complex 
behaviour (Hyperelastic, plastic, etc...). In our case, the real material is silicon which we will approximate 
with an elastic deformation law and simulate using the Finite Element Method (..autolink::General::FEM). In sofa, 
the ..autolink::STLIB::ElasticMaterialObject from *stlib.physics.deformable* provides a scene template
to easily add such an object in your scene. 

To compute the deformation of the object using the Finite Element Method a volumetric representation of shape
must be provided. In our case we are using a tetrahedral mesh stored in a "vtk" file. This can be done with any meshing tool 
such as Gmsh or CGAL but in our case we already provide the tetrahedral mesh in a file name "*finger.vtk*". 

The scene with a finger like soft material may look like:
<div>
```python
from stlib.scene import ..autolink::STLIB::MainHeader
from stlib.physics.deformable import ..autolink::STLIB::ElasticMaterialObject

def createScene(rootNode):
   m=..autolink::STLIB::MainHeader(rootNode, plugins=["SoftRobots"])
   m.getObject("..autolink::Sofa::VisualStyle").displayFlags='showForceFields showBehaviorModels showInteractionForceFields'
 
   finger = ..autolink::STLIB::ElasticMaterialObject(fromVolumeMesh="data/mesh/finger.vtk",
                          ..autolink::STLIB::PARAMS::withYoungModulus=18000,
                          ..autolink::STLIB::PARAMS::withPoissonRatio=0.5,
                          withTotalMass=0.5,
                          attachedTo=rootNode)
```
<div>
<pre>
<a href="details/step2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene.</a>
</pre>
</div>
</div>

If you run the the scene you can see the tetrahedral mesh with blue-ish element. To control the visualization of this computation mesh you can either check the "Force Fields" option within the *View* panel in the runSofa GUI or, as we did, change the displayFlags property of the ..autolink::Sofa::VisualStyle.
In case you want to learn more about volumetric mesh generation there is a dedicated tutorial called
..autolink::SoftRobots::Docs::MeshGeneration.


Instead of displaying the tetrahedron mesh computational structure it is possible to wrap it into a visual model.
This visual model can be of much higher visual quality and is deformed according to the deformation of the underlying mecanical 
mesh. To add such visual model you simply need to add *withSurfaceMesh="data/mesh/finger.stl"*
to the ..autolink::STLIB::ElasticMaterialObject template's arguments.


Here is a set of tasks you can do to get better understanding of this step. 

####<i>Task 2.0:</i>
- In *mycablegripper.pyscn*, replace the lines from previous step with the one needed to create an elastic object with a finger shape. 

####<i>Task 2.1:</i>
- In *mycablegripper.pyscn*, add a visual model to the elastic object you just created. You should get something similar to:
<div>
<pre>
<a href="details/step2.1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Visualize this task.</a>
</pre>
</div>

####<i>Task 2.2:</i>
- In *mycablegripper.pyscn* add a ..autolink::STLIB::FixedBox constraint. This ..autolink::STLIB::FixedBox must be applied to the *finger*, holding one of its extremity so that the object stops free falling  because of gravity.
This should result in the following simulation:
<div>
<pre>
<a href="details/step2.2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Visualize this task (the FixedBox is in red).</a>
</pre>
</div>

####<i>Task 2.3:</i>
- What is the ..autolink::Sofa::VisualStyle used for ? What happens if you remove it from your scene ?

####<i>Task 2.4:</i>
- Gives a small explanation of the role of the poisson ratio parameter. 

####<i>Task 2.5:</i>
- Gives a small explanation of the role of the young modulus parameter. 


## Step 3: Actuating the finger with a cable

In the previous step, we showed how to add a finger shape object made of silicone rubber.
We will now actuate it using a 1d inelastic cable attached to the fingertip and passing through
the object geomertry. Using a specific ScriptController the cable length can be increase or decrease 
at run-time using the ctrl+ and ctrl- touch.

To model the cable you can use the ..autolink::SoftRobots::PullingCable template from
the ..autolink::SoftRobots plugin and use it in the following way:
<div>
```python
### ... similar to previous step ....
from softrobots.actuators import ..autolink::SoftRobots::PullingCable

def createScene(rootNode):
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

The cable has rest lenght that can be changed interactively using a dedicated python object called 
a *PythonScriptController*.
You can create your own controllers by in-heriting from the *Sofa.PythonScriptController* class. As in
<div>
```python
import Sofa
class FingerController(Sofa.PythonScriptController):
    def __init__(self, cable):
        self.cableconstraintvalue = cable.getObject("CableConstraint").findData('value')

    def onKeyPressed(self,c):
        if (c == "+"):
            self.cableconstraintvalue.value =  self.cableconstraintvalue.value[0][0] + 1.
```
</div>
This controller is redefining the *onKeyPressed* behavior and it can can then be attached to objet 
it is supposed to control by doing:
<div>
```python
### ... similar to previous step with the added FingerController ....

def createScene(rootNode):
    ### ... similar to previous step ....

    ## This create a PythonScriptController that permits to programatically implement new behavior
    ## or interactions using the Python programming langage. 
    finger.addObject( FingerController(cable) )
```
</div>

Here is a set of tasks you can do to get better understanding of this step.

####<i>Task 3.0:</i>
- Add to the *mycablegripper.pyscn* the *FingerController* object and connect it to the cable object. 

####<i>Task 3.1:</i>
- Add to the *FingerController* object the release of the cable when the key '-' is pressed.
This should result in a behavior similar to:
<div>
<pre>
<a href="details/step3.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the result of this task</a>
</pre>
</div>

####<i>Task 3.2:</i>
- Add to the *FingerController* some printing in the console indicating that the value has changed. 

## Step 4: Defining self-collision regions
By default Sofa doesn't handle self-collisions because they are very expensive to compute.
The drawback is that this can lead to some undesireable behaviors, i.e. self-interesections, when the object deforms a lot
as illustrated in the figure below.

![Finger with self-intersecting](data/images/NoCollisionRegions.png){width=200}

![Without self-intersection](data/images/CollisionRegions2.png){width=200}


To activate self-collision you need to define speciallly the geometries for which collions are
checked and handled. This can be done by adding ..autolink::STLIB::CollisionMesh as in the following:
<div>
```python
## ... Same imports as in step 3 ...
from stlib.physics.collision import ..autolink::STLIB::CollisionMesh

def createScene(rootNode):
    m=..autolink::STLIB::MainHeader(rootNode, plugins=["SoftRobots"])
    ..autolink::STLIB::ContactHeader(rootNode, alarmDistance=5, contactDistance=4, withFrictionCoef=0.08)

    ## ... Remaining content of step 3 ...
    ## ... the finger
    ## ... the cable..
    ## ... the controller..

    ..autolink::STLIB::CollisionMesh(attachedTo=finger,
             fromSurfaceMesh="data/mesh/fingerCollision_part1.stl", withName="part1", withACollisionGroup=1)

    ..autolink::STLIB::CollisionMesh(attachedTo=finger,
              fromSurfaceMesh="data/mesh/fingerCollision_part2.stl", withName="part2", withACollisionGroup=2)
```
</div>

The ..autolink::STLIB::CollisionMesh are set of triangules, each set can be part of one or multiple 
collision group. By using the *"withACollisionGroup"* option you can specify the group of which the 
..autolink::STLIB::CollisionMesh belong to. Triangle from different collision groupes are tested for 
intersections and contact handling.

Here is a set of tasks you can do to get better understanding of this step.

####<i>Task 4.0:</i>
- Add to your existing *"mycablegripper.pyscn"* the auto-collisions and test the result to be sure it work as in the following simulation
<div>
<pre>
<a href="details/step4.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the result of this task</a>
</pre>
</div>

####<i>Task 4.1:</i>
- Tune the ..autolink::STLIB::ContactHeader parameters to decrease the distance at which self-collision repulsion force are generated. 
<div>
<pre>
<a href="details/step4.1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the result of this task</a>
</pre>
</div>

## Step 5: Duplicating the finger to make a complete gripper
Now we have a working single finger we can duplicate it so that it can match the orginal soft gripper
robot design that contains three of them.

![The soft gripper with three fingers](data/images/gripper.png){with=300}

Before doing that we need to refactor a bit our scene to improve its modularity.
To do that we will make a scene template by moving the finger creation into a separated file called *myfinger.py*.

At this point the *myfinger.py* file should contains something similar to:
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

Now the *"mycablegripper.pyscn"* should be much simpler and only containing:
<div>
```python
from stlib.scene import ..autolink::STLIB::MainHeader, ..autolink::STLIB::ContactHeader
from myfinger import Finger

def createScene(rootNode):
    """This is my first Sofa scene"""
    m=..autolink::STLIB::MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    m.getObject("VisualStyle").displayFlags='showForceFields showBehaviorModels showInteractionForceFields'
    ..autolink::STLIB::ContactHeader(rootNode, alarmDistance=3, contactDistance=2, withFrictionCoef=0.08)

    f1 = Finger(selfNode, "Finger1",
           withRotation=[0, 0, 105],
           withTranslation=[20.0,0.0, 0.0],
           withFixingBox=[-20, -10, 0, 20, 10, 15],
           withPullPointLocation=[3, 10.5, 3])
```
</div>



As usual here is a set of tasks you can do to get better understanding of this step.

####<i>Task 5.0:</i>
- Create the *"myfinger.py"* file with the adequate content and update the file *"mycablegripper.pyscn"* to make use it.
<div>
<pre>
<a href="details/step5.0.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the result of this task</a>
</pre>
</div>

####<i>Task 5.1:</i>
- In *mycablegripper.pyscn*, duplicate the Finger object and change the different location, orientation and fixing point so that it match
the desired robot geometry. Do not forget that you can start Sofa with the '-i' option for an automatic reload of the scene each time the source 
file is changed. 
<div>
<pre>
<a href="details/step5.1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the result of this task</a>
</pre>
</div>


## Step 6: Finalizing the gripper. 

At this step we can now create a file named *gripper.py* that will instanciate the three fingers as a single entity 
and implement a python script script controller to handle the gripper movement.

In *gripper.py* the Gripper creation should look like: 
<div>
```python
import Sofa
from myfinger import Finger

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
            dir = [1.0,0.0,0.0]
        # DOWN key : rear
        elif ord(c)==21:
            dir = [1.0,0.0,0.0]
        # LEFT key : left
        elif ord(c)==18:
            dir = [1.0,0.0,0.0]
        elif ord(c)==20:
            dir = [1.0,0.0,0.0]

        if dir != None:
            ### Apply the displacement to each finger. 
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
</div>
</div>


And right now the set of tasks you can do to get better understanding of this step.

####<i>Task 6.0:</i>
- Create the *mygripper.py* file with the appropriate content and update the *mycablegripper.pyscn* accordingly. 
<div>
<pre>
<a href="details/step6.0.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the result of this task</a>
</pre>
</div>

####<i>Task 6.1:</i>
- Attach the GripperController to the Gripper object. When you press CTRL+UP/DOWN/LEFT/RIGHT what do you observe ? 
<div>
<pre>
<a href="details/step6.1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the result of this task</a>
</pre>
</div>

####<i>Task 6.1:</i>
- Fix the GripperController so that it moves in the proper way according to UP/DOWN/LEFT/RIGHT key pressed.  
<div>
<pre>
<a href="details/step6.2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the result of this task</a>
</pre>
</div>


## Step 7: Finalizing the tutorial by adding rigid objects to your scene

In a similar way to step 0. We will now add rigids objects into *mycablegripper.pyscn*.  
The graspable cube is a bit more complexe as the physics is computed at the center of gravity 
and an inertia matrix need to be provided.

The last set of tasks to do in order to finish this tutorial.

####<i>Task 7.0:</i>
- Add a static Floor object to the scene. 
<div>
<pre>
<a href="details/step7.0.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Visualize the resulting scene.</a>
</pre>
</div>

####<i>Task 7.1:</i>

- Add a rigid cube object to the scene. with the following properties:
```python
totalMass=0.03,
volume=20,
inertiaMatrix=[1000.0,0.0,0.0,0.0,1000.0,0.0,0.0,0.0,1000.0],
```
<div>
<pre>
<a href="details/step7.1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Visualize the resulting scene.</a>
</pre>
</div>

### Conclusion
Congratulation, you completed this tutorial. You are strongly encouraged to pursue with the other tutorial and 
read the thematical documentations.

If you have any comments or suggestions, please submit issues on our ..autolink::github/SoftRobots page. 
