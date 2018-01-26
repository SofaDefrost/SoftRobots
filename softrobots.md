# SoftRobots

SoftRobots is a plugin made by the Defrost research team for the open source simulation framework called Sofa. 
The plugin adds functionnalities to Sofa to ease the modeling, the simulation and the control of soft-robots. 
This plugin is provided as-is under the LGPL Licence.

Thematical documentations:

* T0: How to download and install the plugins 

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
 


Tutorials: 

* Make a soft-gripper actuated by cables. [Start ![](docs/images/outicon.png){height=16}](docs/tutorials/CableGripper-Tutorial/cablegripper.pyscn) 
    
* Make a soft-gripper actuated by pneumatics. [Start ![](docs/images/outicon.png){height=16}](docs/tutorials/PneunetGripper-Tutorial/pneunetgripper.pyscn) 


Scenes examples:

   component

    *   engine

        *   [Finger.pyscn](examples/component/engine/VolumeFromTriangles/Finger.pyscn)

        *   [Finger.pyscn](examples/component/engine/VolumeFromTetrahedrons/Finger.pyscn)

    *   controller

        *   AnimationEditor

            *   [RigidAnimation.pyscn](examples/component/controller/AnimationEditor/RigidAnimation.pyscn)

            *   [Accordion_AnimationEditor.pyscn](examples/component/controller/AnimationEditor/Accordion_AnimationEditor.pyscn)

        *   CommunicationController

            *   [SimulationDirect_Receiver.pyscn](examples/component/controller/CommunicationController/
            SimulationDirect_Receiver.pyscn)

            *   [SimulationInverse_Sender.pyscn](examples/component/controller/CommunicationController/SimulationInverse_Sender.pyscn)

        *   [DataVariationLimiter.pyscn](examples/component/controller/DataVariationLimiter/DataVariationLimiter.pyscn)

    *   constraint

        *   SurfacePressureConstraint

            *   [Springy.pyscn](examples/component/constraint/SurfacePressureConstraint/Springy.pyscn)

            *   [PressureVsVolumeGrowthControl.pyscn](examples/component/constraint/SurfacePressureConstraint/PressureVsVolumeGrowthControl.pyscn)

            *   [SurfacePressureConstraint.pyscn](examples/component/constraint/SurfacePressureConstraint/SurfacePressureConstraint.pyscn)

        *   CableConstraint

            *   [Finger.pyscn](examples/component/constraint/CableConstraint/Finger.pyscn)

            *   [DisplacementVsForceControl.pyscn](examples/component/constraint/CableConstraint/DisplacementVsForceControl.pyscn)

            *   [SoftGripper.pyscn](examples/component/constraint/CableConstraint/SoftGripper.pyscn)

        *   [ArticulatedTentacle.pyscn](examples/component/constraint/UnilateralPlaneConstraint/ArticulatedTentacle.pyscn)*   [softrobots.pyscn](softrobots.pyscn)
