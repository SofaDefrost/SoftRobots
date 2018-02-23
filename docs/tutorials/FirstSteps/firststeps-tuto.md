![](../../images/pluginimage.png){width=100%}



## First steps with Sofa & SoftRobots
Welcome in Sofa and the SoftRobots plugins. This tutorial is aimed at people
that have never use Sofa and aims at give them the basis of Sofa scene modelling.

This tutorial describes how to set-up a simulation environment, a scene, using ..autolink::Sofa and how to use the
..autolink::STLIB plugin to add simulated elements.

Tutorials prequisites:

- installed ..autolink::Sofa with the ..autolink::STLIB.

- you have basic knowledge of the ..autolink::General::Python programming language. If not you can go to ..autolink::General::PythonTutorials.

### Step 1: Setting up simple scene

Sofa is loading the description of the simulation from *pyscn* files. The content of these file is in fact standard python code with
at least one function named *createScene* taking a single parameter, the root of the scene hierarchy. This function is the entry point used by Sofa
to fill the simulation's content and this is the place where you will type your scene's description. A scene is an ordered tree of nodes (ex:gripper.), with parent/child relationship (ex: finger). Each node has one or a few components. Every node and component has a name and a few features. The main node at the top of the tree is called "rootNode".

A very simple scene may look like:
<div>
```python
from stlib.scene import ..autolink::STLIB::MainHeader, ..autolink::STLIB::ContactHeader
from stlib.visuals import ShowGrid
from stlib.physics.rigid import ..autolink::STLIB::Floor
from stlib.physics.rigid import ..autolink::STLIB::Cube

def createScene(rootNode):
    """This is my first scene"""
    ..autolink::STLIB::MainHeader(rootNode, gravity=[0.0,-981.0,0.0])
    ..autolink::STLIB::ContactHeader(rootNode, alarmDistance=15, contactDistance=10)

    ShowGrid(rootNode)

    ..autolink::STLIB::Floor(rootNode,
          withTranslation=[0.0,-160.0,0.0],
          isAStaticObject=True)

    ..autolink::STLIB::Cube(rootNode,
          withTranslation=[0.0,0.0,0.0],
          withScale=20.0)


    return rootNode
```
<div>
<pre>
<a href="details/step1.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Show me the result.</a>
<a href="myproject/firststeps.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Do it yourself.</a>
</pre>
</div>
</div>


####<i>Comments (things to present or do in this step):</i>

- Load the file 'firststeps.pyscn' in runSofa with the -i option and in your text editor.

- The '-i' option is there to automatically reload the file when there is changes.

- Move the object in the scene

- Change their scale

- Explaine the role of the alarmDistance & contactDistance



### Step 2: Adding more elements to the sene

We will now add much more cube falling on the floor.

<div>
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
          withTranslation=[0.0,-160.0,0.0],
          withScale=5.0,
          isAStaticObject=True)

    for c in range(10):
        ..autolink::STLIB::Cube(rootNode,
             withTranslation=[-200+c*50,0.0,0.0],
             withColor=[c/10.0,c*0.7/10.0,0.9],
             withScale=20.0)


    return rootNode
```
<div>
<pre>
<a href="details/step2.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Show me the result.</a>
<a href="myproject/firststeps.pyscn"> <img src="../../images/icons/play.png" width="16px"/>Do it yourself.</a>
</pre>
</div>
</div>


####<i>Comments (things to present or do in this step):</i>

- Add the for loop and change the translation.

- Explain a bit the rendering options to change the color

- Explain that a Floor and Cube are specific cases of a ..autolink::STLIB::RigidObject
