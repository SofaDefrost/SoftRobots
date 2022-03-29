![](docs/images/pluginimage.png){width=100%}

> SoftRobots is a plugin made by the Defrost research team for the open source simulation framework called Sofa.
The plugin adds functionalities to Sofa to ease the modeling, the simulation and the control of soft-robots.
This plugin is provided as-is under the LGPL Licence.

Tutorials:
-----------

* First step for Sofa newcommers: ..autolink::SoftRobots::Docs::FirstStep

* Make a soft-gripper actuated by cables. [Start the tutorial](docs/sofapython3/tutorials/CableGripper/cablegripper-tuto.py)

* Make a soft-gripper actuated by pneumatics. [Start the tutorial](docs/sofapython3/tutorials/PneunetGripper/pneunetgripper-tuto.py)

* Make a soft-tripod actuated by servo motors. [Start the tutorial](docs/sofapython3/tutorials/Tripod/tripod-tuto.py)


Thematical documentations:
-----------

* T0: How to download and install the plugins.

* T1: Elements (tetrahedra, hexahedra, beam).
In this tutorial you will find examples and documentation about the different element types available in Sofa: tetrahedra, hexahedra and beam.

* T2: From mesh to simulation.
In this tutorial we first introduce the tools you can use to generate your meshes, and then how to load your meshes in Sofa (for visualization and simulation).

* T3: Material properties
With Sofa you can simulate both rigid and deformable bodies. When dealing with deformable, you can chose either an elastic material or hyperelastic. In this tutorial we give examples and documentation of each choice.

* T4: Direct actuation
The SoftRobots plugin contains different choices of actuation (eg cable, pressure, hydraulic...). In this tutorial we give documentation and examples of each actuators.
For a forward control of your robot you may want to use keyboard commands to drive the robot. We also provide a "how to" for this feature.

* T5: Open loop control
This tutorial explains how to control your robot (end-effector for example) by solving the inverse problem. This feature is part of the SoftRobots.Inverse plugin which is currently under a private licence.

* T6: Collision

* T7: AccuracyImprovemnt

* T8:ClosedLoop


