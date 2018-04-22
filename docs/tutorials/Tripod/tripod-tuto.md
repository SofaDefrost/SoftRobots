![](../../images/pluginimage.png){width=100%}

<script language="javascript">
function toggle(target) {
    d = document.getElementById(target);
    if(d.className === "show")
        d.className = "hide"
    else 
        d.className = "show"
    return false;
}
</script>


## Simulating a soft robot
This tutorial describes how to set-up a simulation environment, a scene, using ..autolink::Sofa and how to use the
..autolink::SoftRobots plugin to model a virtual soft robot driven by servo motors. Once modeled in Sofa the robot can be simulated and controlled.

Tutorials prequisites:

- installed ..autolink::Sofa with the ..autolink::STLIB and
..autolink::SoftRobots plugins.

- you have basic knowledge of the ..autolink::General::Python programming language. If this is not the case you can go to ..autolink::General::PythonTutorials.

- you have basic knowledge of making scenes with Sofa. If not, please complete the ..autolink::SoftRobots::Docs::FirstStep tutorial first.


The robot we are going to simulate and controlled is acutated by three servo motors connected to a deformable material. 

![](images/tripodPhoto.jpg){width=25%}

Once a scene is loaded in Sofa you can now click on the [Animate] button in the Sofa GUI and start interacting
in the 3D window. When your cursor have clicked on the 3D window you can control the
simulation to manipulate the gripper or interact by pressing CTRL+keys

Note that with MacOS, you may have to use *cmd* instead of *ctrl*.

### Step 1: Making a simple scene with Sofa

Sofa is loading the description of the simulation from *pyscn* files. The content of these file is in fact standard python code with 
at least one function named *createScene* taking a single parameter, the root of the scene hierarchy. This function is the entry point used by Sofa
to fill the simulation's content and this is the place where you will type your scene's description. A scene is an ordered tree of nodes (ex:gripper.), with parent/child relationship (ex: finger). Each node has one or a few components. Every node and component has a name and a few features. The main node at the top of the tree is called "rootNode".

A very simple scene may look like:
<pre>
<a href="details/step1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step1code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step1code' class='hide'>
```python
..autofile::details/step1.pyscn
```
</div>
</div>

####<i>At this step you should be able to:</i>

- Load a scene in sofa with the -i option and open the scene in your text editor.

- Understand that the '-i' allows there to automatically reload the file when there is changes.

- Add a visual object in the scene

- Move the object in the scene

- Be able to visualize in the GUI the different properties of sofa objects

- Know how to change object properties (data)

- Know how to connect the properties from two different object with a link. 

### Step 2: Adding mechanical objects

We will now add a deformable object to a scene. To do that we first need to define a place that will hold the degree of freedom, this place
is called a *MechanicalObject* as in the following scene: 
<pre>
<a href="details/step2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step2code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step2code' class='hide'>
```python
..autofile::details/step2.pyscn
```
</div>
</div>

####<i>At this step you should be able to:</i>

- Add a MechanicalObject

- Add an UniformMass to this MechanicalObject

- Visualize different properties of a MechanicalObject


### Step 3: Modelling deformable material
To implement a mechanical behavior the MechanicalObject must be connected to one or multiple ForceFields. These Forcefieds are in charge 
of computing the internal forces that will guide the deformation of the computation. There exists a lot of different mechanical behavior and it is important to understand the one that approximates the behavior of the real object your want to simulate.In particular, it is important to know how soft or stiff the material is, if it has an elastic or more complex 
behaviour (Hyperelastic, plastic, etc...). In our case, the real material is silicon which we will approximate 
with an elastic deformation law and simulate using the Finite Element Method (..autolink::General::FEM). In sofa, 
the ..autolink::STLIB::ElasticMaterialObject from *stlib.physics.deformable* provides a ready to use prefabricated object
to easily add such an object in your scene. Nevertheless, before using this prefabricated object let's try to build 
our own based on a corotational Finite Element Method with a tetrahedral volumetric representation of the shape. In our case 
we are using a tetrahedral mesh stored in a "gidmesh" file. 

<pre>
<a href="details/step3.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step3code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step3code' class='hide'>
```python
..autofile::details/step3.pyscn
```
</div>
</div>

If you run the the scene you can see the tetrahedral mesh with blue-ish element. To control the visualization
of this computation mesh you can either check the "ForceFields" option within the *View* panel in the runSofa GUI
or, as we did, change the displayFlags property of the ..autolink::Sofa::VisualStyle. 

####<i>At this step you should be able to:</i>

- Understand what a ForceField is

- Use the TetrahedronFEMForceField that provide elastic material behavior based on Finite Element Method 

- Understand how to make a reusable component with function or class. 

- Understand how to search in the automatically generated documentation.

### Step 4: Adding externals constraints

<pre>
<a href="details/step4.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step4code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step4code' class='hide'>
```python
..autofile::details/step4.pyscn
```
</div>
</div>

####<i>At this step you should be able to:</i>

- Understand what is a BoxROI 

- Understand how this Fixing Box ROI is implemented using a RestShapeSpringForceField

- Change the location of the box to constraint a specific location

### Step 5: Adding actuators

It is now time to add the actuators that will deform the elastic shape. This is done by using a prefabricated object 
from actuatedarm.py. 

<pre>
<a href="details/step5.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step5code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step5code' class='hide'>
```python
..autofile::details/step5.pyscn
```
</div>
</div>

####<i>At this step you should be able to:</i>

- Explore the actuatedarm.py & s90servo.py which define the actuators. 

- Explore the Info panel of a component and observe that the prefab have self documentation & specific properties

- Understand the structure of the actuatedarm prefabricated object.

- Understand how the ServoMotor & ServoWheel are interoperating

- Change the initial configuration of the actuatedarm so they constrain a different part of the model 


### Step 6: Adding controllers

The servo motors have a default angular position. To interactively change this default position we will add a dedicated object called a *Controller*. Controllers allow to implement custom behavior and end-user interaction directly using python. 

In this step we are adding such a controller. 

The controller is implementing:

 - CTRL+Key A to move the servo motors from their initial shape to their physical position 

 - CTRL+key up to increase the angle for servo 0

 - CTRL+key down to decrease change the angle for servo 0

 - CTRL+Key left to increase the angle for servo 1 

 - CTRL+Key right to decrease the angle for servo 1 

 - CTRL+Key plus to increase the angle for servo 2

 - CTRL+Key minus to decrease the angle for servo 2

<pre>
<a href="details/step6.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step6code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step6code' class='hide'>
```python
..autofile::details/step6.pyscn
```
</div>
</div>

### Step 7: Connecting to the  physical world. 
It is now time to connect our simulated robot to the real one. To do that we need to add two things. The first one,  *SerialPortBridgeGeneric*, is a component to handle the communication throught the USB/Serial port, the second one, *SerialPortController*,  is reading the angle from the servo motor's data field to actually send them to the communication layer.

The controller in the scene are now implementing:

 - CTRL+Key A to move the servo motors from their initial shape to their physical position

 - CTRL+Key B to start sending data to the real robot

 - CTRL+key up to increase the angle for servo 0

 - CTRL+key down to decrease change the angle for servo 0

 - CTRL+Key left to increase the angle for servo 1 

 - CTRL+Key right to decrease the angle for servo 1 

 - CTRL+Key plus to increase the angle for servo 2

 - CTRL+Key minus to decrease the angle for servo 2

<pre>
<a href="details/step7.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step7code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step6code' class='hide'>
```python
..autofile::details/step7.pyscn
```
</div>
</div>


### Step 8: Adding collision

By default Sofa doesn't handle collision as they are very expensive to compute. To activate collisions you need to define specially
the geometries for which collions are checked and how they are handled. In this step we are adding a rigid Sphere object falling on the robots. By default there is no collision so we need to specifically activate that by adding 'Contact' to the existing scene. 

A new controller, called JumpController*, is also added to change rapidely the servo motors angles so the robots can plays with the 
falling ball.

The controller in the scene are now implementing:

 - CTRL+Key A to move the servo motors to their physical position 

 - CTRL+Key Q to move the servo motors to an intermediate angle position

 - CTRL+Key Z to move the servo motors to an high angular position

 - CTRL+Key  to start sending data to the real robot

 - CTRL+key up to increase the angle for servo 0

 - CTRL+key down to decrease change the angle for servo 0

 - CTRL+Key left to increase the angle for servo 1 

 - CTRL+Key right to decrease the angle for servo 1 

 - CTRL+Key plus to increase the angle for servo 2

 - CTRL+Key minus to decrease the angle for servo 2

<pre>
<a href="details/step8.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step8code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step8code' class='hide'>
```python
..autofile::details/step8.pyscn
```
</div>
</div>


### Step 9: Inverse control
In the previous steps we where controlling  the robot by directly specifying the angle of the ServorMotor object. In this step we will use Sofa to inverse the model and adding an effector to the simulation so that it become possible to specify the effector position and let the simulation computes the angles to reach the effectors's position. 

<pre>
<a href="Rigidification/TripodRigidInverse.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step9code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step9code' class='hide'>
```python
..autofile::Rigidification/TripodRigidInverse.pyscn
```
</div>
</div>



### Conclusion
Congratulation, you completed this tutorial. You are strongly encouraged to pursue with the other tutorial and
read the thematical documentations.

If you have any comments or suggestions, please submit issues on our ..autolink::github/SoftRobots page.
