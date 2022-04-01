# Tetrahedron topology for FEM simulation

For the FEM simulation in Sofa the domain can be either a Tetrahedra (often just called 'Tetra') or Hexahedra ('Hexa') mesh. This example shows a robot that has been meshed into Tetra mesh. For more on how to generate this kind of mesh see also Step 1 in the [Cable Gripper tutorial]( ../../T2-FromMeshToSimulation/index.html)

After the mesh has been created a 'TetrahedronFEMForceField' can be applied to the mesh, with the two main properties: the Poisson Ratio and the Young Modulus of the material:

~~~ {.python}
robot.createObject('TetrahedronFEMForceField', poissonRatio='0.45',  youngModulus='600');
~~~	


After clicking 'Animate', you can interact with the scene by using shift + *mouse left click*, which will generate a force that pulls on the robot. 
