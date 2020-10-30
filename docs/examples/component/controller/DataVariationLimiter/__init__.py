# -*- coding: utf-8 -*-
"""

The **DataVariationLimiter** component is used to avoid big variation of an input data. It interpolates between two consecutive inputs when a jump is detected.
In this directory you will find one example showing how to use the component:

- **DataVariationLimiter.pyscn** : Soft actuated accordion

Below are images of the simulation.

.. image:: http://project.inria.fr/softrobot/files/2016/02/DataStabilizer_0.png
   :width: 110px
.. image:: http://project.inria.fr/softrobot/files/2016/02/DataStabilizer_1.png
   :width: 110px
.. image:: http://project.inria.fr/softrobot/files/2016/02/DataStabilizer_2.png
   :width: 110px
.. image:: http://project.inria.fr/softrobot/files/2016/02/DataStabilizer_3.png
   :width: 110px
.. image:: http://project.inria.fr/softrobot/files/2016/02/DataStabilizer_4.png
   :width: 110px
.. image:: http://project.inria.fr/softrobot/files/2016/02/DataStabilizer_5.png
   :width: 110px


Example
*******

.. sourcecode:: python

    	goal = rootNode.createChild('goal')
	goal.createObject('EulerImplicitSolver')
	goal.createObject('CGLinearSolver', iterations='100', tolerance="1e-5", threshold="1e-5")
	goal.createObject('MechanicalObject', name='goalMO',
		          position='0 0 5',
		          showObject="1",
		          showObjectScale="1",
		          drawMode="1")
	goal.createObject('DataVariationLimiter', name="stabilizer", listening="1", input="@goalMO.position") 
	goal.createObject('MechanicalObject', name='goalMOStabilized', 
		          position='@stabilizer.output',
		          showObject="1",
		          showObjectScale="1",
		          drawMode="1")
	goal.createObject('UncoupledConstraintCorrection')


Data fields
***********

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Required
     - Description
   * - **input**
     - Link to the input variables
   * - **output**
     - Link to the output
   * - **size**
     - Input size.
   * - **maxJump**
     - Maximal jump allowed. Default 10% is equivalent to jump = 0.1.
   * - **nbStep**
     - Number of interpolation steps. Default is 50.

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Optional
     - Description
   * - **initOutput**
     - If true, will initialize the output with the input.

"""


