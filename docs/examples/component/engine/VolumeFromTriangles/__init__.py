# -*- coding: utf-8 -*-
"""

The **VolumeFromTriangles** component computes the volume of a given surfacic mesh (triangles or/and quads).
In this directory you will find one example showing how to use the component:

- **Finger.pyscn**

.. image:: https://project.inria.fr/softrobot/files/2016/02/Finger_VolumeTriangle.png
   :width: 50%
   :align: center



Example
*******

.. sourcecode:: python

    	finger = rootNode.createChild('finger')
 
	# Create the popology
	finger.createObject('MeshSTLLoader', name='loader', filename=path+'finger.stl')
	finger.createObject('MeshTopology', src='@loader', name="mesh")
	finger.createObject('VolumeFromTriangles')


Data fields
***********

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Required
     - Description
   * - **positions**
     - If not set by user, find the context mechanical.
   * - **triangles**
     - If not set by user, find the context topology.
   * - **quads**
     - If not set by user, find the context topology.

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Optional
     - Description
   * - **update**
     - If true, will update the volume with the current positions.

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Properties
     - Description
   * - **volume**
     - Read only. Output volume. Only relevant if closed surface.

"""


