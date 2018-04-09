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

## First steps with Sofa & SoftRobots
Welcome in Sofa and the SoftRobots plugins. This tutorial is aimed at people
that have never use Sofa and aims at quickly give them the basis of scene modelling with Sofa.

This tutorial describes how to set-up a simulation environment, a scene, using ..autolink::Sofa and how to use the
..autolink::STLIB plugin to add simulated elements.

Tutorials prequisites:

- installed ..autolink::Sofa with the ..autolink::STLIB.

- you have basic knowledge of the ..autolink::General::Python programming language. If not you can go to ..autolink::General::PythonTutorials.

### Step 1: Setting up simple scene

Sofa is loading the description of the simulation from *pyscn* files. The content of these file is in fact standard python code with
at least one function named *createScene* taking a single parameter, the root of the scene hierarchy. This function is the entry point used by Sofa
to fill the simulation's content and this is the place where you will type your scene's description. A scene is an ordered tree of nodes (ex:gripper.), with parent/child relationship (ex: finger). Each node has one or a few components. Every node and component has a name and a few features. The main node at the top of the tree is called "rootNode".

Making a very simple scene:
<div>
<pre>
<a href="details/step1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/firststeps.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step1code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
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

####<i>At this step you should be able to:</i>

- Load a scene in sofa 'firststeps.pyscn' in runSofa with the -i option and open it in your text editor.

- Understand that the '-i' allows there to automatically reload the file when there is changes.

- Add a visual object in the scene

- Move the object in the scene

- Be able to visualize in the GUI the different properties of sofa object


### Step 2: Make a template to reuse the object you made

<div>
<pre>
<a href="details/step2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/firststeps.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step2code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step2code' class='hide'>
```python
from stlib.scene import ..autolink::STLIB::MainHeader, ..autolink::STLIB::ContactHeader
from stlib.visuals import ShowGrid
from stlib.physics.rigid import ..autolink::STLIB::Floor
from stlib.physics.rigid import ..autolink::STLIB::Cube

def createScene(rootNode):
    """This is my first scene"""
    ..autolink::STLIB::MainHeader(rootNode, gravity=[0.0,-981.0,0.0])
    ..autolink::STLIB::ContactHeader(rootNode, alarmDistance=2, contactDistance=1)

    ShowGrid(rootNode)

    ..autolink::STLIB::Floor(rootNode,
          translation=[0.0,-160.0,0.0],
          uniformScale=5.0,
          isAStaticObject=True)

    for c in range(10):
        ..autolink::STLIB::Cube(rootNode,
             translation=[-200+c*50,0.0,0.0],
             color=[c/10.0,c*0.7/10.0,0.9],
             uniformScale=20.0)


    return rootNode
```
</div>

####<i>At this step you should be able to:</i>

- Be able to make a template made with several components and create multiple instance of this template.


### Step 3: Add mechanical objects.

As in the example scene make a cube falling on the floor.

<div>
<pre>
<a href="details/step3.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Try the scene in Sofa.</a>
<a href="myproject/firststeps.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Write it yourself.</a>
<a href="javascript:void" onclick="toggle('step3code');"> <img src="../../images/icons/play.png" width="16px"/>Show/Hide the code.</a>
</pre>
<div id='step3code' class='hide'>
```python
from stlib.scene import ..autolink::STLIB::MainHeader, ..autolink::STLIB::ContactHeader
from stlib.visuals import ShowGrid
from stlib.physics.rigid import ..autolink::STLIB::Floor
from stlib.physics.rigid import ..autolink::STLIB::Cube
from stlib.physics.deformable import ..autolink::STLIB::ElasticMaterialObject

def createScene(rootNode):
    """This is my first scene"""
    ..autolink::STLIB::MainHeader(rootNode, gravity=[0.0,-981.0,0.0])
    ..autolink::STLIB::ContactHeader(rootNode, alarmDistance=2, contactDistance=1)

    ShowGrid(rootNode)

    ..autolink::STLIB::Floor(rootNode,
          translation=[0.0,-160.0,0.0],
          uniformScale=5.0,
          isAStaticObject=True)

    for c in range(10):
        ..autolink::STLIB::Cube(rootNode,
             translation=[-200+c*50,0.0,0.0],
             color=[c/10.0,c*0.7/10.0,0.9],
             uniformScale=20.0)


    return rootNode
```
</div>


