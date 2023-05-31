# -*- coding: utf-8 -*-
"""
In this directory you will find one example showing how to use the **UnilateralPlaneConstraint** component:

- **ArticulatedTentacle.pyscn** : Soft cable-driven tentacle with self-collision regions

This component is a simple point plane collision model. By providing 4 points to the component, the first point will be constrained to stay in one side of the plane described by the three other points (in the direction of the plane normal). All the four points, the triangle and the normal can be seen by allowing the 'Collision Model' in the 'View' tab. 
Below are images of the simulation. 

.. image:: http://project.inria.fr/softrobot/files/2016/10/ArticulatedTentacle_1-254x300.png
   :height: 175px
.. image:: http://project.inria.fr/softrobot/files/2016/10/ArticulatedTentacle_2-254x300.png
   :height: 175px
.. image:: http://project.inria.fr/softrobot/files/2016/10/ArticulatedTentacle_3-254x300.png
   :height: 175px
.. image:: http://project.inria.fr/softrobot/files/2016/10/ArticulatedTentacle_00000001-300x236.png
   :height: 175px

Example
*******

.. sourcecode:: python

		tentacleContact = tentacle.createChild('contact')
                tentacleContact.createObject('MechanicalObject',
                        position="64 0 11   69 7 8     69 -7 8     71 0 17 "+
                                 "107 0 -23   111 7 -27     111 -7 -27    117 0 -17 "+
                                 "93 0 -7.5   97 7 -11     97 -7 -11    102 0 -0.5 "+
                                 "138 0 -73   141 7 -77     141 -7 -77     146 0 -72 "+
                                 "78 0 3     83 7 0    83 -7 0    86 0 9   "+
                                 "118 0 -38   122 6.7 -42     122 -7 -42   129 -0.2 -35  "+
                                 "129.5 0 -55.5    132 7 -60    132.5 -7 -59.6   138 0 -53.5")
                tentacleContact.createObject('UnilateralPlaneConstraint', indices="0 1 2 3")
                tentacleContact.createObject('UnilateralPlaneConstraint', indices="4 5 6 7")
                tentacleContact.createObject('UnilateralPlaneConstraint', indices="8 9 10 11")
                tentacleContact.createObject('UnilateralPlaneConstraint', indices="12 13 14 15")
                tentacleContact.createObject('UnilateralPlaneConstraint', indices="16 17 18 19")
                tentacleContact.createObject('UnilateralPlaneConstraint', indices="20 21 22 23")
                tentacleContact.createObject('UnilateralPlaneConstraint', indices="24 25 26 27")
                tentacleContact.createObject('BarycentricMapping')


Data fields
***********

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Required
     - Description
   * - **indices**
     - Four indices: First one for the constrained point. The others to describe the plane.

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Optional
     - Description
   * - **flipNormal**
     - The normal must be to the direction of the point.

"""


