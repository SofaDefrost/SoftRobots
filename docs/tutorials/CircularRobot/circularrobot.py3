import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'


def Target(parentNode):
    target = parentNode.addChild('Target')
    target.addObject('EulerImplicitSolver', firstOrder=True)
    target.addObject('CGLinearSolver', iterations=100, tolerance=1e-4, threshold=1e-4)
    target.addObject('MechanicalObject', name='dofs', showObject=True, showObjectScale=5, drawMode=1, position=[0, 0, 5])
    target.addObject('UncoupledConstraintCorrection')
    return target

def Floor(parentNode, color=[0.5, 0.5, 0.5, 1.], rotation=[90, 0, 180], translation=[50, -21, -100]):
    floor = parentNode.addChild('Floor')
    floor.addObject('MeshObjLoader', name='loader', filename='mesh/square1.obj', scale=250, rotation=rotation, translation=translation)
    floor.addObject('OglModel', src='@loader', color=color)
    floor.addObject('MeshTopology', src='@loader', name='topo')
    floor.addObject('MechanicalObject')
    floor.addObject('TriangleCollisionModel')
    floor.addObject('LineCollisionModel')
    floor.addObject('PointCollisionModel')
    return floor

def Wall(parentNode, color=[0.5, 0.5, 0.5, 1.], rotation=[0, 0, 0], translation=[-200, -271, 100]):
    wall = parentNode.addChild('Wall')
    wall.addObject('MeshObjLoader', name='loader', filename='mesh/square1.obj', scale=250, rotation=rotation, translation=translation)
    wall.addObject('OglModel', src='@loader', color=color)
    return wall

class CircularRobot():
    """ This prefab is implementing a soft circular robot actuated with four cables.

        The prefab is composed of:
        - a visual model
        - a collision model
        - a mechanical model for the deformable structure

        The prefab has the following parameters:
        - youngModulus
        - poissonRatio
        - totalMass

        Example of use in a Sofa scene:

        def createScene(root):
            ...
            circularrobot = CircularRobot(root)
    """

    def __init__(self, parentNode, youngModulus=500, poissonRatio=0.3, totalMass=0.042, inverseMode=True, effectorTarget=None):
        self.inverseMode = inverseMode

        self.node = parentNode.addChild('CircularRobot')
        self.node.addObject('MeshVTKLoader', name="loader", filename=path+"wheel.vtk")
        self.node.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra="@loader.tetrahedra")
        self.node.addObject('TetrahedronSetTopologyModifier')
        self.node.addObject('MechanicalObject', name='dofs', showIndices=False, showIndicesScale=4e-5)
        self.node.addObject('UniformMass', totalMass=totalMass)
        self.node.addObject('TetrahedronFEMForceField', poissonRatio=poissonRatio,  youngModulus=youngModulus)

        if inverseMode:
            if effectorTarget is None:
                Sofa.msg_warning("The prefab CircularRobot needs effectorTarget in inverseMode")
            self.node.addObject('BarycentricCenterEffector', limitShiftToTarget=True, maxShiftToTarget=5,
                                    effectorGoal=effectorTarget,
                                    axis=[1, 1, 1])
        self.__addCables()

    def __addCables(self):
        cables=["16 0 5","-16 0 5","0 16 5","0 -16 5","11 11 5","-11 -11 5","11 -11 5","-11 11 5"]
        for i in range(0,4):
            cable = self.node.addChild('cable'+str(i+1))
            cable.addObject('VisualStyle', displayFlags="showInteractionForceFields")
            cable.addObject('MechanicalObject' , position=cables[i*2]+" "+cables[i*2+1])
            cable.addObject('CableActuator' if self.inverseMode else 'CableConstraint', name="cable", indices=[0, 1], hasPullPoint=0, minForce=0, maxPositiveDisp=12, maxDispVariation=1.5)
            cable.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)

    def addVisualModel(self, color=[1., 0., 0., 1.]):
        modelVisu = self.node.addChild('VisualModel')
        modelVisu.addObject('MeshSTLLoader', filename=path+'wheel_visu.stl')
        modelVisu.addObject('OglModel', color=color)
        modelVisu.addObject('BarycentricMapping')

    def addCollisionModel(self):
        modelContact = self.node.addChild('CollisionModel')
        modelContact.addObject('MeshSTLLoader', name='loader', filename=path+'wheel_collision.stl', scale=1)
        modelContact.addObject('MeshTopology', src='@loader', name='topo')
        modelContact.addObject('MechanicalObject')
        modelContact.addObject('TriangleCollisionModel')
        modelContact.addObject('LineCollisionModel')
        modelContact.addObject('PointCollisionModel')
        modelContact.addObject('BarycentricMapping')


def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SoftRobots.Inverse SofaOpenglVisual SofaSparseSolver SofaImplicitOdeSolver SofaLoader SofaMeshCollision SofaSimpleFem SofaConstraint')
    rootNode.addObject('VisualStyle', displayFlags='hideWireframe showVisualModels showBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields hideInteractionForceFields')
    rootNode.findData('gravity').value = [0, -9180, 0]
    rootNode.findData('dt').value = 0.01

    # Add solver for inverse resolution
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', epsilon=2e-0, maxIterations=2500, tolerance=1e-7, responseFriction=0.8)

    # Contact detection methods
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase', name="N2")
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response="FrictionContact", responseParams="mu=0.8")
    rootNode.addObject('LocalMinDistance', alarmDistance=5, contactDistance=1, angleCone=0.0)

    # Add linear solver
    simulation = rootNode.addChild("Simulation")
    simulation.addObject('EulerImplicitSolver', rayleighMass=0.015, rayleighStiffness=0.015)
    simulation.addObject('SparseLDLSolver')
    simulation.addObject("GenericConstraintCorrection", solverName="SparseLDLSolver")

    Floor(rootNode)
    target = Target(rootNode)

    circularrobot = CircularRobot(simulation, effectorTarget=target.dofs.getData("position").getLinkPath())
    circularrobot.addVisualModel()
    circularrobot.addCollisionModel()


    return rootNode
