#Displacement vs. Force Control for Cables

In this scene we demonstrate how to use the _CableConstraint_ component to model an inelastic 1d cable. This cable is embedded in an elastic finger. The cable is free to slide through the material. You can either control the force (left) applied by the cable or its displacement (right). If the force is controlled (for example, with a DC motor), the simulation will solve for the displacement that satisfies that force condition. If the displacement is controlled (for example, with a servo motor), the simulation will solve for the force that is required to obtain that displacement. The cable can only apply force in tension.

<div>
<pre>
<img src="images/DisplacementVsForceControl.png" />
</pre>
</div>
