# T4-DirectActuation with Python Controller


To add script that will be executed during the animation of our scene we use the component called **_PythonScriptController_**.
We call it controller because it will allow use to recuperate data of component during the animation to modify and actualize the components data.

>During this tutorial we will show you how to use this component to control via KeyBoard the movement of a robot.

## Instanciation


In our scene we need first to add this line in the beginning :
```
rootNode.createObject('RequiredPlugin', pluginName='SofaPython')
```
 
Because this *PythonScriptController* comes from the plugin **_SofaPython_** which is already compiled and comes with our plugin **_SoftRobots_**.

Now we can now add our *PythonScriptController* component in our SOFA scene :
```
rootNode.createObject('PythonScriptController', classname="controller", filename="controlKeyboard.py")
```
  
there are 2 arguments to this component :
- **_classname_** : corespond to the name of our function 
- **_filename_** : corespond to the path (?) and name of our python file

>Now that we have instanciate it in our sofa scene we have to make the actual script to execute during animation time.

## Control our robot


With the previous instanciation we now have to create a file *controlKeyboard*.*py* that will implement a function called *controller*.

first we have to import Sofa library :

    import Sofa

Then declare our class :

    class controller(Sofa.PythonScriptController):

#### Bindings
***

> This *PythonScriptController* allows **bindings** between SOFA and python so our python script can use standard function name like **initGraph** which will then be executed with all the other SOFA component during initialization and animation.

Here we will use two binded functions :
- **_initGraph(self , node)_** : 

    in this function we can initialize all the variables our class will need and it will be instanciate during SOFA initialization phase with all the other components of our SOFA scene
    
- **_onKeyPressed(self , c)_** : 

    this function is executed when **ctrl + any KeyBoard key** is pressed. We recuparate the key **c** and if it's the right key trigger a response.

To see all the other possible functions you can use, go to **LINK_TO_SofaPython_DOC**.

#### Description of initGraph
***

We define the links to the current SOFA **node**, our **increment** to move the robots and initialize two list : 
- **restPositionNodes** that will contains the links to the actuator objects we want to modify.
- **initPose** that will contains the first position allowing us to come back easily to it.  

```
def initGraph(self, node):
    self.node = node
    self.increment = 5
    self.restPositionNodes = []
    self.initPose = []
```
We fill *restPositionNodes* with the links to our actuators:
```
    for i in range(3):
        print 'RestPositionLeg'+str(i)
        self.restPositionNodes.append(self.node.getChild('RestPositionLeg'+str(i))) 
```
We fill *initPose* with the initial values :
```
    for i in range(len(self.restPositionNodes)):
        self.initPose.append(self.restPositionNodes[i].getObject('meca'+str(i)).translation[0])
```

#### Description of onKeyPressed
***

On key pressed we first have to get the current values of our actuator position :
```
def onKeyPressed(self,c):
    
    currentValues = []
    for i in range(len(self.restPositionNodes)):
        currentValues.append(self.restPositionNodes[i].getObject('meca'+str(i)).translation[0])
```
Then depending of the key pressed, we -/+ increment the actuator position. 
We decided to have 6 keys : 
- 3 for each direction
- 1 for all up
- 1 for all down
- 1 to reset to the initial values

```
    if (ord(c) == 18):  #  <--
        currentValues[0][2] += self.increment
        currentValues[1][2] += self.increment
        currentValues[2][2] -= self.increment
    ...
    # Same things for the other 5 key we have
```
And finally we can update the new position we have into the actuator :
```
for i in range(len(self.restPositionNodes)):
    self.restPositionNodes[i].getObject('meca'+str(i)).findData("translation").value =  currentValues[i]
```
