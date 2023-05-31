# -*- coding: utf-8 -*-
"""
In this directory you will find multiple examples showing how to use the **SurfacePressureConstraint** component:

- **Springy.pyscn** : Soft actuated accordion
- **PressureVsVolumeGrowthControl.pyscn** : Stanford bunny 

Below is a video of a soft Standford bunny actuated with pressure in its inner cavity. You can run this simulation by loading the file **PressureVsVolumeGrowthControl.pyscn** with the application runSofa.

.. raw:: html

   <p align=center><iframe src="https://www.youtube.com/embed/Vc0fRwo2RhM" width="270" height="190" frameborder="0" allowfullscreen="allowfullscreen"></iframe></iframe></p>

Example
*******

.. sourcecode:: python

        #  This create a new node in the scene. This node is appended to the accordion's node.
	cavity = accordion.createChild('cavity')

	# This create a MechanicalObject, a component holding the degree of freedom of our
	# mechanical modelling. In the case of a pneumatic actuation it is a set of positions describing the cavity wall.
	cavity.createObject('MeshSTLLoader', name='loader', filename=path+'Springy_Cavity.stl')
	cavity.createObject('MeshTopology', src='@loader', name='topo')
	cavity.createObject('MechanicalObject', name='cavity')

	# Create a SurfacePressureConstraint object with a name.
	cavity.createObject('SurfacePressureConstraint', template='Vec3', name="pressure",
			    triangles='@topo.triangles',
			    valueType="1",
			    value="8")

	# This create a BarycentricMapping. A BarycentricMapping is a key element as it will create a bi-directional link
	# between the cavity wall (surfacic mesh) and the accordion (volumetric mesh) so that movements of the cavity's DoFs will be mapped
	# to the accordion and vice-versa;
	cavity.createObject('BarycentricMapping', name='mapping',  mapForces=False, mapMasses=False)



Data fields
***********

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Required
     - Description
   * - **triangles**
     - List of triangles on which the surface pressure is applied. If no list is given, the component will fill the two lists with the context topology.
   * - **quads**
     - List of quads on which the surface pressure is applied. If no list is given, the component will fill the two lists with the context topology.
   * - **value**
     - List of choices for volume growth or pressure to impose.
   * - **valueIndex**
     - Index of the value (in InputValue vector) that we want to impose. If unspecified the default value is {0}.
   * - **valueType**
     - Either "volumeGrowth", the contstraint will impose the volume growth provided in data value[valueIndex], or "pressure", in this case the contstraint will impose the pressure provided in data value[valueIndex]. If unspecified, the default value is pressure.


.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Optional
     - Description
   * - **flipNormal**
     - Allows to invert cavity faces orientation. If a positive pressure acts like a depressurization, try to set flipNormal to true.
   * - **maxPressure**
     - Maximum pressure allowed for actuation. If no value is set by user, no maximum pressure constraint will be considered.
   * - **minPressure**
     - Minimum pressure allowed for actuation. If no value is set by user, no minimum pressure constraint will be considered. A negative pressure will empty/drain the cavity.
   * - **maxVolumeGrowth**
     - Maximum volume growth allowed for actuation. If no value is set by user, no maximum will be considered. NB: this value has a dependancy with the time step (volume/dt) in the dynamic case.
   * - **minVolumeGrowth**
     - Minimum volume growth allowed for actuation. If no value is set by user, no minimum will be considered. NB: this value has a dependancy with the time step (volume/dt) in the dynamic case.
   * - **maxVolumeGrowthVariation**
     - Maximum volume growth variation allowed for actuation. If no value is set by user, no maximum will be considered. NB: this value has a dependancy with the time step (volume/dt) in the dynamic case.
   * - **drawPressure**
     - Visualization of the value of pressure. If unspecified, the default value is {false}.
   * - **drawScale**
     - Scale for visualization. If unspecified the default value is {0.1}.

.. list-table:: 
   :header-rows: 1
   :widths: auto

   * - Properties
     - Description
   * - **volumeGrowth**
     - Read only. Output volume growth.
   * - **pressure**
     - Read only. Output pressure.
   * - **initialCavityVolume**
     - Read only. Output volume of the cavity at init (only relevant in case of closed mesh).
   * - **cavityVolume**
     - Read only. Output volume of the cavity (only relevant in case of closed mesh).

"""


