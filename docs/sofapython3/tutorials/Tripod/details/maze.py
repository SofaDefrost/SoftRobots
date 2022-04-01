import Sofa
import Sofa.Core
from stlib3.scene.contactheader import ContactHeader
from mazecontroller import MazeController


class Maze(Sofa.Prefab):

    properties = [
        {'name':'name',           'type':'string', 'help':'Node name',                  'default':'Maze'},
        {'name':'index',          'type':'int', 'help':'index of rigid to attach to',   'default':0},
        {'name':'translation',    'type':'Vec3d', 'help':'',                            'default':[-50,5,50]},
        {'name':'rotation',       'type':'Vec3d', 'help':'',                            'default':[-90,0,0]}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.addObject("MeshSTLLoader", name="loader", filename="data/mesh/maze/maze_4_coarse.stl",
                       translation=self.translation.value, rotation=self.rotation.value)
        self.addObject("MeshTopology", src='@loader')
        self.addObject("MechanicalObject")
        self.addObject("TriangleCollisionModel")
        self.addObject("LineCollisionModel")
        self.addObject("PointCollisionModel")


class Sphere(Sofa.Prefab):

    properties = [
        {'name':'name',        'type':'string', 'help':'Node name',  'default':'Sphere'},
        {'name':'position',    'type':'Vec3d', 'help':'',             'default':[-22,50,-27]}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.addObject('EulerImplicitSolver')
        self.addObject('SparseLDLSolver')
        self.addObject("MechanicalObject", position=self.position.value)
        self.addObject("UniformMass", totalMass=0.001)
        self.addObject('SphereCollisionModel', radius=2)
        self.addObject('GenericConstraintCorrection')


def createScene(rootNode):

    rootNode.gravity=[0., -9810., 0.]
    rootNode.dt=0.01
    ContactHeader(rootNode, alarmDistance=15, contactDistance=0.5, frictionCoef=0)
    rootNode.addObject('VisualStyle', displayFlags=['showCollisionModels', 'showBehavior'])
    rootNode.addObject('DefaultVisualManagerLoop')

    effector = rootNode.addChild('Effector')
    effector.addObject('EulerImplicitSolver', firstOrder=True)
    effector.addObject('CGLinearSolver', iterations=100, threshold=1e-5, tolerance=1e-5)
    effector.addObject('MechanicalObject', template='Rigid3', name='goalMO', position=[0,40,0,0,0,0,1], showObject=True, showObjectScale=10)
    effector.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
    effector.addObject('UncoupledConstraintCorrection', compliance='1e-10  1e-10  0 0 1e-10  0 1e-10 ')
    effector.addObject(MazeController(effector, True))

    maze = effector.addChild(Maze())
    maze.addObject("RigidMapping", index=0)

    rootNode.addChild(Sphere())

    return
