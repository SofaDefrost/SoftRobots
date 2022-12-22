from stlib3.scene import MainHeader
from stlib3.physics.rigid import Floor


def createScene(rootNode):

    pluginList = ["Sofa.Component.Collision.Geometry",
                  "Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Iterative",
                  "Sofa.Component.Mass",
                  "Sofa.Component.ODESolver.Backward",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Constant",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D"]

    # A default gravity force is implemented on Sofa. Here we reset it,
    # choosing millimeters as the length unit for the scene.
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=pluginList)
    rootNode.VisualStyle.displayFlags = 'showCollisionModels'

    cube = rootNode.addChild("Cube")

    # Mechanical model

    totalMass = 1.0
    volume = 1.0
    inertiaMatrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    cube.addObject('MechanicalObject', name="DOF", template="Rigid3", showObject=True, showObjectScale=30.0,
                   translation=[0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0])
    cube.addObject('UniformMass', name="vertexMass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    # The following line defines the material behaviour when submitted to constraints;
    # it is not necessary here, as no interaction between objects has been defined
    # cube.addObject('UncoupledConstraintCorrection')

    # Time integration and solver

    cube.addObject('EulerImplicitSolver', name='odesolver')
    cube.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-5, threshold=1e-5)

    # Visual Object of the Cube

    visual = cube.addChild("CubeVisual")
    # Graphic model based on a mesh
    visual.addObject("MeshOBJLoader", name="loader", filename="mesh/smCube27.obj")
    visual.addObject('OglModel', name="Visual", src="@loader", color=[0.1, 0.0, 1.0], scale=20.0)
    # Building a correspondence between the mechanical and the graphical representation
    visual.addObject('RigidMapping')

    # Adding the Floor for more fun ;)
    Floor(rootNode,
          translation=[0.0, -300.0, 0.0],
          uniformScale=5.0,
          isAStaticObject=True)

    return rootNode
