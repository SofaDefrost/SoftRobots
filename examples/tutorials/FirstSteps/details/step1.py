from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.rigid import Floor, Cube


def createScene(rootNode):
    """This is my first scene"""
    rootNode.addObject("VisualGrid", nbSubdiv=10, size=1000)

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
                  "Sofa.GL.Component.Rendering3D",
                  "Sofa.Component.Constraint.Lagrangian.Correction"]

    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=pluginList)
    ContactHeader(rootNode, alarmDistance=15, contactDistance=10)
    rootNode.VisualStyle.displayFlags = "showCollisionModels"

    Floor(rootNode, translation=[0.0, -160.0, 0.0],
          isAStaticObject=True)

    cube = Cube(rootNode, translation=[0.0, 0.0, 0.0],
                uniformScale=20.0)

    cube.addObject('UncoupledConstraintCorrection')

    return rootNode
