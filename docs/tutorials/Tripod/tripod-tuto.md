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

The robot we are going to simulate is acutated by three servo motors connected to a deformable material. 

![](images/tripodPhoto.jpg){width=25%}

Tutorials prequisites:

- installed ..autolink::Sofa with the ..autolink::STLIB and
..autolink::SoftRobots plugins.

- you have basic knowledge of the ..autolink::General::Python programming language. If this is not the case you can go to ..autolink::General::PythonTutorials.

- you have basic knowledge of making scenes with Sofa. If not, please complete the ..autolink::SoftRobots::Docs::FirstStep tutorial first.


### Step 1: Making a simple scene with Sofa

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

Controllers allow to implement custom behavior and user interaction in python. In this step we are adding such a controller 
name MyController. This implemented behavior is that the ServoMotor angles when the user is pressing keys up, down, left, right, + and -.  When the user is pressing the spacebar an animation is started that moves the robots from its a flat configuration to one that match the ones of the fabricated robot. 

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

####<i>At this step you should be able to:</i>

- Add your own controller

- handle user input

- start an animation 

### Step 7: Adding collision
In this step we are adding a rigid Sphere object falling on the robots. By default there is no collision so we need to specifically activate that by adding 'Contact' to the existing scene. 

<pre>
<a href="details/step7.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step7code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step7code' class='hide'>
```python
..autofile::details/step7.pyscn
```
</div>
</div>



### Step 8: Inverse control
In the previous steps we where controlling  the robot by directly specifying the angle of the ServorMotor object. In this step we will use Sofa to inverse the model and adding an effector to the simulation so that it become possible to specify the effector position and let the simulation computes the angles to reach the effectors's position. 

<pre>
<a href="Rigidification/TripodRigidInverse.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/tripod.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step8code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step8code' class='hide'>
```python
..autofile::Rigidification/TripodRigidInverse.pyscn
```
</div>
</div>



### Conclusion
Congratulation, you completed this tutorial. You are strongly encouraged to pursue with the other tutorial and
read the thematical documentations.

If you have any comments or suggestions, please submit issues on our ..autolink::github/SoftRobots page.
