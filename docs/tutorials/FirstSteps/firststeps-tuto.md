![](../../images/pluginimage.png){width=100%}

..autolink::SoftRobots::TODO-BECAUSE-NOTHING-IS-DONE

## First steps
Welcome in Sofa and the SoftRobots plugins. This tutorial is aimed at people 
that have never use Sofa to give them the basis to understand the other tutorials
of the SoftRobots plugins. 

We provide a myproject.zip with some basic element. Un-compress this folder which should contain:
- myproject.pyscn, this is the file loaded that sofa that will create the scene 
- cubes.py, this file contains 
- data/images/

### Step 0: Load the scene in Sofa
To execute the scene you need to type runSofa myproject.pyscn 


####<i>Task 1.2:</i>
- Add a rigid Cube. The cube must be of size '20'.

####<i>Task 1.3:</i>
- Add a serie of 5 cubes in a row with equidistant spacing and different color.
(Note, with Sofa colors are encoded as a triplet of float encoding bewteen 0.0 and 1.0 the intensity of the Red/Green/Blue component. a ..autolink::STLIB::Floor is a specific kind of ..autolink::STLIB::RigidObject thus it have the same parameters).
