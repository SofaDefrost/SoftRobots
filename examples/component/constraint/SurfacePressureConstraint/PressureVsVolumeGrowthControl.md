# Pressure vs. volume growth actuation 

This example illustrates the difference between pressure and volume growth actuation. Pressure based actuation represents an actuation performed by compressing a gas (air). Volume based actuation represents an actuation performed by an incompressible medium (water, oil).

FYI: If the computation is slow, it might be because you did not add sparse and metis to sofa configuration.</i></p>	

		
The scene simulate a pressure on a cavity in the bunny body. You can either control the volume growth (right) or the pressure (left). It uses the _SurfacePressureConstraint_ component, available in the SoftRobots plugin
Further descriptions of the component can be found at:
https://project.inria.fr/softrobot/documentation/
		