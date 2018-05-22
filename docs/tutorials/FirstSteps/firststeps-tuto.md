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

<!-- dégager les ContactHeader sauf dernière étape -->

## First steps with Sofa & SoftRobots
Welcome to Sofa and the SoftRobots plugins. This tutorial is intended for people
who have never used Sofa, and aims at providing them quickly with the basis of scene modelling with Sofa.

This tutorial describes how to set up a simulation environment, a scene, using ..autolink::Sofa and how to use the Sofa Template Library
..autolink::STLIB to add simulated elements.

Tutorials prequisites:

- ..autolink::Sofa and the library ..autolink::STLIB are installed.

- you have basic knowledge of the ..autolink::General::Python programming language. If not, you can go to ..autolink::General::PythonTutorials.

<!-- ### Step 1: Loading a scene on Sofa

Sofa is loading the description of the simulation from *pyscn* files. Sofa is started with the command `runSofa`{.bash} in the terminal.  
To run a file `MyScene.pyscn`, use the following command:

```bash
PATH_TO_SOFA_DIRECTORY/build/bin/runSofa PATH_TO_SCENE_FILE/MyScene.pyscn
```
In order to be able to send data to a connected robot, one way is to start Sofa with administrator rights, using the command `sudo` (it requires the administrator password):  

```bash
sudo PATH_TO_BUILD_DIRECTORY/bin/runSofa PATH_TO_SCENE/MyScene.pyscn
```
-->

### Step 1: Setting up a simple scene

####<i>At the end of this step, you will be able to:</i>
- Write a simple scene in Sofa using built-in objects called *prefabs* (or *templates*)
- Modify the properties of these objects
- Conveniently reload the scene after each modification

The content of the `pyscn` simulation files is in fact standard python code with at least one function named `createScene` taking a single parameter, the root of the scene hierarchy. This function is the entry point used by Sofa to fill the simulation's content and this is the place where you will type your scene's description.  
A scene is an ordered tree of nodes, representing objects (example of node: gripper), with parent/child relationship (example of gripper's child: finger). Each node has one or more components. Every node and component has a name and a few features. The main node at the top of the tree is called "rootNode". Additional components can be added to the scene, that aren't nodes (they cannot have children), related to the behaviour of the object (examples: *UniformMass* for mass parameters definition, and *OGLModel* for the settings of the graphic display). 

Making a very simple scene:
<div>
<pre>
<a href="details/step1.pyscn"> <img src="../../images/icons/play.png" width="14px"/> Try the scene in Sofa.</a>
<a href="myproject/firststeps.pyscn"> <img src="../../images/icons/play.png" width="14px"/> Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step1code');"> <img src="../../images/icons/play.png" width="14px"/> Show/Hide the code.</a>
</pre>
<div id='step1code' class='hide'>
```python
from stlib.scene import ..autolink::STLIB::MainHeader, ..autolink::STLIB::ContactHeader
from stlib.visuals import ShowGrid
from stlib.physics.rigid import ..autolink::STLIB::Floor
from stlib.physics.rigid import ..autolink::STLIB::Cube

def createScene(rootNode):
    """This is my first scene"""
    ..autolink::STLIB::MainHeader(rootNode, gravity=[0.0,-981.0,0.0])
    ..autolink::STLIB::ContactHeader(rootNode, alarmDistance=15, contactDistance=10)

    ..autolink::STLIB::Floor(rootNode,
          translation=[0.0,-160.0,0.0],
          isAStaticObject=True)

    ..autolink::STLIB::Cube(rootNode,
          translation=[0.0,0.0,0.0],
          uniformScale=20.0)


    return rootNode
```
</div>
</div>

####<i>Remarks</i>
- The main node (rootNode) in this scene has two child nodes: Floor and Cube (the two physical objects present in the scene).
- The rootNode includes two behaviour descriptions: `MainHeader` (defining gravity as the main force exercised on the objects, assuming the length is in millimeters) and `ContactHeader` (stating how a contact beween the objects is handled: here the objects mustn't be able to go through one another). These behaviours apply to all its child nodes.
- Both the Cube and the Floor are built-in objects, *prefabs*, which means that they are already implemented simulation models, including components and child nodes.

####<i>Exploring the scene</i>

- The [*Animate*] button allows to start and stop playing the scene.

- All scene codes can be modified: right click anywhere in the *Graph* panel of the Sofa GUI, and click on *Open file in editor* in the dropdown menu. The modifications need to be saved ([*Save*] button) before reloading the scene. 

- In order to reload the scene (after each modification of the code), press *Ctrl+R* or select *File \> Reload* in the menu bar.

- To automatically reload the scene when there are changes, add the option `-i` when first loading the scene in the terminal: `runSofa firststeps.pyscn -i`{.bash}.

- In order to vizualize the properties of the objects directly from the GUI, double-click on the wanted item in the *Graph* panel to open the corresponding settings window. The properties can be modified directly from this window (click on the [*Update*] button to reload the scene with the new parameters afterwards).

You can try the following manipulations, in order to get familiar with Sofa environment:  
(Click on the text to Show/Hide the solution)

<div>
<pre>
<a href="javascript:void" onclick="toggle('step1exo');"> <img src="../../images/icons/play.png" width="14px"/>Change the position of the cube from the Sofa GUI</a>
<a href="javascript:void" onclick="toggle('step1exo2');"> <img src="../../images/icons/play.png" width="14px"/>Change the color of the cube, directly in the code</a>
</pre>
<div id='step1exo' class='hide'>
In the *Graph* panel on the left, unroll the 'Cube' menu and double-click on 'MechanicalObject mstate'.  
In the window that appears, go to the *Transformation* tab: the line 'translation' allows you to move the object in the scene.
</div>
<div id='step1exo2' class='hide'>
After having opened the code file, add the `color` argument to the `Cube` object.  
The function becomes 
```python
Cube(rootNode,
      translation=[0.0,0.0,0.0],
      uniformScale=20.0,
      color=[0.0,0.0,1.0])
```  
The color vector is defined by percentages of [Red,Green,Blue].  
Don't forget to save and reload the scene.
</div>
</div>
<!-- ça marche, voir si on veut mettre le txt de la solution en forme ou pas-->


### Step 2: Building a Mechanical model for an object simulation & its Visual model
<!-- titre à revoir -->

####<i>At the end of this step, you will be able to:</i>
- Build a simple mechanical model for an object
- Build a corresponding visual object
- Add time integration and solving tools
- Understand the necessity for a collision management model


Both the Cube and the Floor objects used in Step 2 are actually built-in objects called *prefabs*. In the following steps, a deeper insight into Sofa's object modeling is provided. The next two steps aim at receating the *prefab* Cube used in Step 2. (For a more dynamic scene, the Floor prefab is still present.) 
First of all, a mechanical model of the object is built. For the purpose of simulation, the object is discretized in space: it is divided into small volumes connected together by points (called nodes).  
The aim of the simulation is to compute, at each time step, the next position and velocity of each of these nodes, based on the forces they are subjected to.  
Each of these points represents a degree of freedom (DOF) of the object. All the positions and velocities of these points are stored in what is called the *MechanicalObject*.

```python
cube.createObject('MechanicalObject', name, template, translation, rotation)
```  

The physical properties of the object material, like its mass distribution, are also implemented. 

```python
cube.createObject('UniformMass', name, mass=[totalMass, volume, inertiaMatrix[:]])
```

A time integration scheme is then added and defines the system to be solved at each time step of the simulation (here the implicit Euler Method). A solving method is in turn added (here the Conjugate Gradient method), that solves the equations governing the model at each time step, and updates the *MechanicalObject*.  

This model alone is enough to run the simulation of the Cube's fall under gravity force. However, to be able to view it on screen, a visual model of the object must be created. This is one of the child *nodes* of the object. The virtual object is modeled with graphic vectors: the volume of the object is, here again, discretized. The resulting set of points and their connections to each other (vectors) is called the *mesh*. Figure 1 below shows the initial mesh, composed of tetrahedra.
<figure>
  <img class="centered" src="CubeMeshViewer.png" alt="" width="200px"/>
  <figcaption>Figure 1: A view of the cube's mesh as described in the file <i>smCube27.obj</i></figcaption>
</figure>
At each time step, the *MechanicalObject* undergoes modifications (of its position, speed ...).  
Finally, in order to match the visual representation and the mechanical one, a mapping tool is implemented: it builds the correspondance between the points of the MechanicalObject and the nodes of the mesh.

<div>
<pre>
<a href="details/step2.pyscn"> <img src="../../images/icons/play.png" width="14px"/>Try the scene in Sofa.</a>
<a href="myproject/firststeps.pyscn"> <img src="../../images/icons/play.png" width="14px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step2code');"> <img src="../../images/icons/play.png" width="14px"/>Show/Hide the code.</a>
</pre>
<div id='step2code' class='hide'>
```python
from stlib.scene import MainHeader
from stlib.visuals import ShowGrid
from stlib.solver import DefaultSolver
from stlib.physics.rigid import Floor

def createScene(rootNode):
  ShowGrid(rootNode)

  # A default gravity force is implemented on Sofa. Here we reset it, choosing millimeters as the length unit for the scene.
  MainHeader(rootNode, gravity=[0.0,-981.0,0.0])

  cube = rootNode.createChild("Cube")

  ### Mechanical model

  totalMass = 1.0
  volume = 1.0
  inertiaMatrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


  cube.createObject('MechanicalObject', name="DOF", template="Rigid", translation=[0.0,0.0,0.0], rotation=[0.0,0.0,0.0])
  cube.createObject('UniformMass', name="mass", mass=[totalMass, volume, inertiaMatrix[:]])

  # The following line defines the material behaviour when submitted to constraints; it is not necessary in this step, as no interaction between objects has been defined
  #cube.createObject('UncoupledConstraintCorrection')

  ### Time integration and solver

  cube.createObject('EulerImplicit', name='odesolver')
  cube.createObject('CGLinearSolver', name='Solver')


  ### Visual Object of the Cube

  visual = cube.createChild("Cube Visual")
  # Graphic model based on a mesh
  visual.createObject('OglModel', name="Visual", fileMesh="mesh/smCube27.obj", color=[0.1,0.0,1.0], scale=20.0)
  # Building a correspondance between the mechanical and the graphical representation
  visual.createObject('RigidMapping')

  ########################################
  ### Adding the Floor for more fun ;) ###
  Floor(rootNode,
          translation=[0.0,-300.0,0.0],
          uniformScale=5.0,
          isAStaticObject=True)


  return rootNode
```
</div>
</div>

####<i>Remarks</i>
- The points of the mesh are called nodes, but this term has nothing to do with the *nodes* of Sofa, related to the hierarchy of the objects.
- The objects simulated in this tutorial are rigid. The additional components describing the internal forces of deformable objects won't be discussed in this introduction tutorial.

####<i>Exploring the scene</i>
By clicking on the [*Animate*] button, the Cube can be seen falling endlessly, due to gravity force. It even goes through the Floor as if it were a ghost. The reason for this behaviour is that the two objects of the scene (the Cube and the Floor) have been modeled separately. No line code refers to the behaviour to adopt when they collide.


### Step 3: Adding interactions between objects - collision modeling.

####<i>At the end of this step, you will be able to:</i>
- Add a collision model to the objects in a scene
- Understand the multi-model representation of the objects in Sofa

In order to make objects interact with each other, a *collision* model is required. The collision model is another child node of the Cube. Collisions are handled with yet another MechanicalObject representing the Cube, and specifications on how the Cube should react to a contact: here the contact is stopping it from moving further. This is implemented with the following line from Step 2, now uncommented:

```python
cube.createObject('UncoupledConstraintCorrection')
```

The constraints of the collision apply on the surface on the Cube. As it is decomposed into tetrahedral solid elements, the surface elements are either triangles, lines or points. They represent the degrees of freedom of the collision model. The surface mesh is described based on the one used in the Visual model.  
Finally, in order to map those collision DOFs with those of the mechanical model, a `RigidMapping` is used here as well.  

<div>
<pre>
<a href="details/step3.pyscn"> <img src="../../images/icons/play.png" width="14px"/>Try the scene in Sofa.</a>
<a href="myproject/firststeps.pyscn"> <img src="../../images/icons/play.png" width="14px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step3code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step3code' class='hide'>
```python  
from stlib.scene import MainHeader, ContactHeader
from stlib.visuals import ShowGrid
from stlib.solver import DefaultSolver
from stlib.physics.rigid import Floor

def createScene(rootNode):
  ShowGrid(rootNode)

  # A default gravity force is implemented on Sofa. Here we reset it, choosing millimeters as the length unit for the scene.
  MainHeader(rootNode, gravity=[0.0,-981.0,0.0])

  cube = rootNode.createChild("Cube")

  ### Mechanical model

  totalMass = 1.0
  volume = 1.0
  inertiaMatrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


  cube.createObject('MechanicalObject', name="DOF", template="Rigid", translation=[0.0,0.0,0.0], rotation=[0.0,0.0,0.0])
  cube.createObject('UniformMass', name="mass", mass=[totalMass, volume, inertiaMatrix[:]])

  # Material behaviour when submitted to constraints
  cube.createObject('UncoupledConstraintCorrection')

  ### Time integration and solver

  cube.createObject('EulerImplicit', name='odesolver')
  cube.createObject('CGLinearSolver', name='Solver')


  ### Visual Object of the Cube

  visual = cube.createChild("Cube Visual")
  # Graphic model based on a mesh
  visual.createObject('OglModel', name="Visual", fileMesh="mesh/smCube27.obj", color=[0.1,0.0,1.0], scale=20.0)
  # Building a correspondance between the mechanical and the graphical representation
  visual.createObject('RigidMapping')

  ### Collision Object for the Cube

  collision = cube.createChild("Cube Collision Model")
  collision.createObject('MeshObjLoader', name="loader", filename="mesh/smCube27.obj", triangulate="true", scale=20.0)

  collision.createObject('Mesh', src="@loader")
  collision.createObject('MechanicalObject')

  collision.createObject('Triangle')
  collision.createObject('Line')
  collision.createObject('Point')

  collision.createObject('RigidMapping')


  ########################################
  ### Adding the Floor for more fun ;) ###
  floor = Floor(rootNode,
      name="Floor",
          translation=[0.0,-300.0,0.0],
          uniformScale=5.0,
          isAStaticObject=True)


  #Collision buil-in function already used in Step 1
  ContactHeader(floor, alarmDistance=10, contactDistance=5)

  return rootNode
```
</div>
</div>

####<i>Exploring the scene</i>
- Thanks to the collision model, the Cube's fall is now stopped by the Floor.  
- With the addition of the collision model, there are now three representations of the same Cube object: a mechanical one, a visual one, and a collision model representation (see Figure 2 below). This multi-model representation is characteristic of Sofa, along with the mapping functions to build correspondance between the representations.
<figure>
  <img class="centered" src="MultiModalRep.png" alt="" width="600px"/>
  <figcaption>Figure 2: Multi-modal representation of the cube</i></figcaption>
</figure>  
- These representations are accessible via the *View* panel of the Sofa GUI.


####<i>Remarks</i>
- The code that was built in the last two steps in order to implement the Cube model constitutes the `Cube()` prefab. Such prefab objects allow great time savings, and a lighter code.  
- The function `ContactHeader` used for the Floor collision model is composed of the same elements as the Cube collision model that was built in this step, grouped in one function.  

### Step 4: Use *prefabs* to quickly model more complex scenes
<!-- CUBE CODE: plugins/STLIB/python/stlib/physics/rigid/rigidobject.py -->

####<i>At the end of this step, you will be able to:</i>
- Build a more complex scene with prefabs
- Use a loop structure to build several instances of the same object

Based on the prefab object `Cube()` and `Foor()`, as well as the collision management function `ContactHeader()`, the aim of this step is to build rapidly the scene on Figure 3 below:  
<figure>
  <img class="centered" src="result_step4.png" alt="" width="400px"/>
  <figcaption>Figure 3: Initial view of the simulation</i></figcaption>
</figure>

<div>
<pre>
<a href="details/step4.pyscn"> <img src="../../images/icons/play.png" width="14px"/>Try the scene in Sofa.</a>
<a href="myproject/firststeps.pyscn"> <img src="../../images/icons/play.png" width="14px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step4code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step4code' class='hide'>
```python  
from stlib.scene import MainHeader, ContactHeader
from stlib.visuals import ShowGrid
from stlib.physics.rigid import Floor
from stlib.physics.rigid import Cube

def createScene(rootNode):
    """This is my first scene"""
    MainHeader(rootNode, gravity=[0.0,-981.0,0.0])
    ContactHeader(rootNode, alarmDistance=8, contactDistance=5)

    ShowGrid(rootNode)

    Floor(rootNode,
          translation=[0.0,-160.0,0.0],
          uniformScale=5.0,
          isAStaticObject=True)
    
    Floor(rootNode,
           name="FloorObstacle",
             translation=[0.0,-80.0,0.0],
             color=[0.0,1.0,0.0],
             uniformScale=0.8,
             isAStaticObject=True)

    for c in range(7):
        Cube(rootNode,
       name="Cube"+str(-210+c*70),
             translation=[-210+c*70,0.0,0.0],
             color=[c/10.0,c*0.7/10.0,0.9],
             uniformScale=20.0)



    return rootNode
```
</div>
</div>

####<i>Exploring the scene</i>
- All seven cubes are similar in size and two of their coordinates are identical. A variable `c` is defined, in order to modify the translation parameter and the color of each cube. By using a loop structure, the Cube prefab function can be written only once.
- The collision model of this scene is more complex: collisions are possible between the cubes, between the central cubes and the intermediate green floor, and finally between the cubes and the yellow floor. The function `ContactHeader` applies to all the child nodes of the rootNode, governing the whole collision possibilities with one call of the function in the rootNode.  

### Conclusion
Congratulations, you completed this tutorial! You are strongly encouraged to pursue with the other tutorials and read the thematical documentations.

If you have any comments or suggestions, please submit them on our github/SoftRobots page.