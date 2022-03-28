# CableConstraint

Actuating the robot with a displacement-controlled cable.
----------------

Here we have a finger-shaped soft robot made of a deformable material. We actuate the finger by pulling on a cable embedded in the material. The cable can slide through the material. We can directly actuate the displacement of the cable, so the length of the cable is changed.

In order to control the finger, you can use ctrl+ and ctrl-.

To create a cable actuator for direct control, we will use the CableConstraint element and the following steps.
1. Create a new node for the cable. This should be a child of the node containing the main body that the cable will interact with.
2. Create a MechanicalObject by listing the position of the points that the cable should pass through.
3. Create a CableConstraint.
	* In the 'indices' field, list the points that the cable should pass through by listing their index in the MechanicalObject in the desired connection order. While we use all of the points in the MechanicalObject in order in this example, this is not necessary.
	* By default, the CableConstraint is controlled by displacement. This can be done explicitly by specifying valueType="displacement" when creating the CableConstraint.
	* The pull point is a static point from which the cable originates.
4. Create a BarycentricMapping for the cable. This creates a bidirectional link between the cable and finger degrees of freedom. This allows the two bodies to interact. By default, this maps this node's MechanicalObject DOFs to the DOFs of the MechanicalObject in the parent node.

Information on how to create a PythonScriptController to control the actuators via the keyboard can be found in this example: [driveTheRobot](../../DriveTheRobot/Simulation.py)

Instead of controlling the cable length, you can control the force applied from the cable pull point. You can see this in the scene [CableForce.py3scn](CableForce.py).
