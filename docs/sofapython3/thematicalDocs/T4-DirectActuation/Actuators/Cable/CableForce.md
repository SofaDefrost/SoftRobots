# CableForce

Actuating the robot with a force-controlled cable.

This scene is identical to [CableDisplacement.py](CableDisplacement.py) except that the cable here is force-controlled rather than displacement controlled. To understand more about direct cable actuation, see that example first. 
In order to control the cable via force instead of displacement, we use the valueType field when defining the CableConstraint, setting it to 'force'.
