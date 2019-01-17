# -*- coding: utf-8 -*-
"""
Cable constraint

In this directory you will find multiple examples showing how to use the CableConstraint component:

- **Finger.pyscn** : Soft actuated finger
- **CableConstraint.pyscn** : Stanford bunny 
- **DisplacementVsForceControl.pyscn** : Stanford bunny 

Below is a video of a soft finger actuated with one cable. You can run this simulation by loading the file **Finger.pyscn** with the application runSofa.

.. raw:: html

   <p align=center><iframe src="https://www.youtube.com/embed/uXkxI6NwIIY" width="270" height="190" frameborder="0" allowfullscreen="allowfullscreen"></iframe></p>

**Data fields:**

.. list-table:: 
   :header-rows: 0
   :widths: auto

   * - **indices** 
     - List of points connected by the cable (from extremity to actuated point). If no indices are given, default value is 0. In case of multiple indices, one point will be actuated and the others will represent sliding points for the cable.
   * - **pullPoint** 
     - Fixed point from which the cable is pulled. If unspecified, the default value is {0.0,0.0,0.0}

"""


