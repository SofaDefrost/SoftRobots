from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.rigid import Floor
from stlib3.physics.rigid import Cube


def createScene(rootNode):
    """This is my first scene"""

    pluginList = ["Sofa.Component.AnimationLoop",
                  "Sofa.Component.Collision.Detection.Algorithm",
                  "Sofa.Component.Collision.Detection.Intersection",
                  "Sofa.Component.Collision.Geometry",
                  "Sofa.Component.Collision.Response.Contact",
                  "Sofa.Component.Constraint.Lagrangian.Solver",
                  "Sofa.Component.IO.Mesh",
                  "Sofa.Component.LinearSolver.Iterative",
                  "Sofa.Component.Mass",
                  "Sofa.Component.ODESolver.Backward",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Constant",
                  "Sofa.Component.Visual",
                  "Sofa.GL.Component.Rendering3D"]

    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=pluginList)
    ContactHeader(rootNode, alarmDistance=15, contactDistance=10)

    Floor(rootNode,
          translation=[0.0, -160.0, 0.0],
          isAStaticObject=True)

    Cube(rootNode,
         translation=[0.0, 0.0, 0.0],
         uniformScale=20.0)

    return rootNode
