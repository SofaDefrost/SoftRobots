![](../../images/pluginimage.png){width=100%}

## Simulating a soft robot
This tutorial describes how to set-up a simulation environment, a scene, using ..autolink::Sofa and how to use the
..autolink::SoftRobots plugin to model a virtual soft robot driven by servo motors.
Once modeled in Sofa the robot can be simulated and controlled.

Tutorials prequisites:

- installed ..autolink::Sofa with the ..autolink::STLIB and
..autolink::SoftRobots plugins.

- you have basic knowledge of the ..autolink::General::Python programming language. If this is not the case you can go to ..autolink::General::PythonTutorials.

- you have basic knowledge of making scenes with Sofa. If not, please complete the ..autolink::SoftRobots::Docs::FirstStep tutorial first.


### Step 0: Try the simulation in Sofa

### Step 1: Visual model
#    Scene vide
#    Ajouter un noeud (ElasticBody)
#       Ajouter un loader => (surfacique .stl)
#       Ajouter visual => createObject(""), expliquer le GUI, expliquer les liens

### Bilan... on switch sur le concept de Prefab. Dans la suite vous aller utiliser ce genre de Prefab

## Step 2: modélisation des contraines internes
#       Expliquer DOF puis ForceField.

## Step 3
#       Expliquer FEM ... mesh
#       Ajouter un mapping qui relie la méca au  visuel.

## Step 4: modélisation des contraintes externes (fixed boxes)

## Step 5: on utilise les prefab... ActuatedArm
#   import actuatedart
#   Accrocher les servo au maillage déformable

## Step 6: la collision ?  (POURQUOI LA COLLISION)
#    Ajouter un objet rigide qui tombe...
#    Ajouter le modèle de collision xxxx


## Step 5: on ajoute le controleur ...animation soit du clavier.

## Step 6: on conecte au hardware réel ?

##



### Conclusion
Congratulation, you completed this tutorial. You are strongly encouraged to pursue with the other tutorial and
read the thematical documentations.

If you have any comments or suggestions, please submit issues on our ..autolink::github/SoftRobots page.
