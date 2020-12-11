# -*- coding: utf-8 -*-
"""

The **VolumeFromTetrahedrons** component computes the volume of a given volumetric mesh (tetrahedrons or/and hexahedrons).
In this directory you will find one example showing how to use the component:

- **Finger.pyscn**

.. image:: https://project.inria.fr/softrobot/files/2016/02/Finger_Volume.png
   :width: 50%
   :align: center

Example
*******

.. sourcecode:: python

    	finger = rootNode.createChild('finger')

	# Create the popology
	finger.createObject('MeshVTKLoader', name='loader', filename=path+'finger.vtk')
	finger.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
	finger.createObject('TetrahedronSetTopologyModifier')

	finger.createObject('VolumeFromTetrahedrons')


Data fields
***********

.. list-table::
   :header-rows: 1
   :widths: auto

   * - Required
     - Description
   * - **positions**
     - If not set by user, find the context mechanical.
   * - **tetras**
     - If not set by user, find the context topology.
   * - **hexas**
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
     - Read only. Output volume.

"""
