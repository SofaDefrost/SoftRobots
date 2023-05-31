# -*- coding: utf-8 -*-
"""

In this directory you will find multiple examples showing how to use the **CableConstraint** component:

- **Finger.py** : Soft actuated finger
- **FingerWithSTLIB.py** : Soft actuated finger using the STLIB plugin
- **DisplacementVsForceControl.py** : Soft actuated fingers showing different controls

Below is a video of a soft finger actuated with one cable. You can run this simulation by loading the file **Finger.pyscn** with the application runSofa.

.. raw:: html

   <p align=center><iframe src="https://www.youtube.com/embed/uXkxI6NwIIY" width="270" height="190" frameborder="0" allowfullscreen="allowfullscreen"></iframe></p>


Example
*******

.. sourcecode:: python

    	# This create a new node in the scene. This node is appended to the finger's node.
        cable = finger.addChild('cable')

        # This create a MechanicalObject, a component holding the degree of freedom of our
        # mechanical modelling. In the case of a cable it is a set of positions specifying
        # the points where the cable is passing by.
        cable.addObject('MechanicalObject',
                        position=[
                        [-17.5, 12.5, 2.5],
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
                        [-17.5, 12.5, 12.5])

        # Create a CableConstraint object with a name.
        # The indices are referring to the MechanicalObject's positions.
        # The last index is where the pullPoint is connected.
        cable.addObject('CableConstraint', name="aCableActuator",
                       #indices=range(0,14),
                       indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
                       pullPoint=[0.0, 12.5, 2.5])

        # This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
        # between the cable's DoFs and the finger's ones so that movements of the cable's DoFs will be mapped
        # to the finger and vice-versa;
        cable.addObject('BarycentricMapping')

Data fields
***********

.. list-table::
   :header-rows: 1
   :widths: auto

   * - Required
     - Description
   * - **indices**
     - List of points connected by the cable (from extremity to actuated point). If no indices are given, default value is 0. In case of multiple indices, one point will be actuated and the others will represent sliding points for the cable.
   * - **pullPoint**
     - Fixed point from which the cable is pulled. If unspecified, the default value is {0.0,0.0,0.0}
   * - **value**
     - Displacement or force to impose.
   * - **valueIndex**
     - Index of the value (in InputValue vector) that we want to impose. If unspecified the default value is {0}.
   * - **valueType**
     - Either "displacement", the contstraint will impose the displacement provided in data value[valueIndex], or force, in this case the contstraint will impose the force provided in data value[valueIndex]. If unspecified, the default value is displacement.

.. list-table::
   :header-rows: 1
   :widths: auto

   * - Optional
     - Description
   * - **maxForce**
     - Maximum force of the actuator. If unspecified no maximum value will be considered.
   * - **minForce**
     - Minimum force of the actuator. If unspecified no minimum value will be considered and the cable will then be seen as a stiff rod able to push.
   * - **maxPositiveDisp**
     - Maximum displacement of the actuator in the positive direction. If unspecified no maximum value will be considered.
   * - **maxNegativeDisp**
     - Maximum displacement of the actuator in the negative direction. If unspecified no maximum value will be considered.
   * - **maxDispVariation**
     - Maximum variation of the displacement allowed. If not set, no max variation will be concidered.
   * - **drawPullPoint**
     - If true, will draw the pull point (default true).
   * - **drawPoints**
     - If true, will draw the points (default true).
   * - **color**
     - Color of the string.
   * - **hasPullPoint**
     - If false, the pull point is not considered and the cable is entirely mapped. In that case, needs at least 2 different point in indices

.. list-table::
   :header-rows: 1
   :widths: auto

   * - Properties
     - Description
   * - **cableInitialLength**
     - Read only. Gives the initial length of the cable
   * - **cableLength**
     - Read only. Gives the current length of the cable. Computation done at the end of the time step.
   * - **force**
     - Read only. Output force
   * - **displacement**
     - Read only. Output displacement compared to the initial cable length

"""
