import Sofa
import Sofa.Core
from stlib3.scene.contactheader import ContactHeader
from mazecontroller import MazeController
import json


class Maze(Sofa.Prefab):
    prefabParameters = [
        {'name': 'translation', 'type': 'Vec3d', 'help': '', 'default': [0, 5, 0]},
        {'name': 'rotation', 'type': 'Vec3d', 'help': '', 'default': [-90, 0, 0]}
    ]

    prefabData = [
        {'name': 'index', 'type': 'int', 'help': 'index of rigid to attach to', 'default': 0},
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.addObject("MeshSTLLoader", name="loader", filename="data/mesh/maze/maze.stl",
                       translation=self.translation.value, rotation=self.rotation.value)
        self.addObject("MeshTopology", src='@loader')
        self.addObject("MechanicalObject")
        self.addObject("TriangleCollisionModel")
        self.addObject("LineCollisionModel")
        self.addObject("PointCollisionModel")


class Sphere(Sofa.Prefab):
    prefabParameters = [
        {'name': 'position', 'type': 'Vec3d', 'help': '', 'default': [3, 50, -8]},
        {'name': 'withSolver', 'type': 'bool', 'help': '', 'default': False}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        if self.withSolver.value:
            self.addObject('EulerImplicitSolver')
            self.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
            self.addObject('GenericConstraintCorrection')
        self.addObject("MechanicalObject", position=self.position.value)
        self.addObject("UniformMass", totalMass=1e-4)
        self.addObject('SphereCollisionModel', radius=2)


def createScene(rootNode):
    rootNode.gravity = [0., -9810., 0.]
    rootNode.dt = 0.01

    pluginList = ["Sofa.Component.AnimationLoop",
                  "Sofa.Component.Collision.Detection.Algorithm",
                  "Sofa.Component.Collision.Detection.Intersection",
                  "Sofa.Component.Collision.Geometry",
                  "Sofa.Component.Collision.Response.Contact",
                  "Sofa.Component.Constraint.Lagrangian.Correction",
                  "Sofa.Component.Constraint.Lagrangian.Solver",
                  "Sofa.Component.IO.Mesh", "Sofa.Component.LinearSolver.Direct",
                  "Sofa.Component.LinearSolver.Iterative", "Sofa.Component.Mass",
                  "Sofa.Component.ODESolver.Backward",
                  "Sofa.Component.SolidMechanics.Spring",
                  "Sofa.Component.StateContainer",
                  "Sofa.Component.Topology.Container.Constant",
                  "Sofa.Component.Visual",
                  "Sofa.Component.Mapping.NonLinear"]

    rootNode.addObject('RequiredPlugin', pluginName=pluginList)

    ContactHeader(rootNode, alarmDistance=15, contactDistance=0.5, frictionCoef=0)
    rootNode.addObject('VisualStyle', displayFlags=['showCollisionModels', 'showBehavior'])
    rootNode.addObject('DefaultVisualManagerLoop')

    effector = rootNode.addChild('Effector')
    effector.addObject('EulerImplicitSolver', firstOrder=True)
    effector.addObject('CGLinearSolver', iterations=100, threshold=1e-5, tolerance=1e-5)
    effector.addObject('MechanicalObject', template='Rigid3', name='goalMO', position=[0, 40, 0, 0, 0, 0, 1],
                       showObject=True, showObjectScale=10)
    effector.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
    effector.addObject('UncoupledConstraintCorrection', compliance='1e-10  1e-10  0 0 1e-10  0 1e-10 ')

    # Open maze planning from JSON file
    data = json.load(open('mazeplanning.json'))
    effector.addObject(MazeController(effector, data["anglePlanningTable"], True))

    maze = effector.addChild(Maze())
    maze.addObject("RigidMapping", index=0)

    rootNode.addChild(Sphere(withSolver=True))

    return
