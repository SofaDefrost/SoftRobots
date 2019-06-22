from math import cos
from math import sin
from splib.objectmodel import SofaPrefab, SofaObject
from splib.numerics import Vec3, Quat
from splib.animation import animate, AnimationManager

import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'
dirPath = os.path.dirname(os.path.abspath(__file__))+'/'


def effectorTarget(parentNode, position=[0., 0., 200]):
    target = parentNode.createChild("Target")
    target.createObject("EulerImplicitSolver", firstOrder=True)
    target.createObject("CGLinearSolver")
    target.createObject("MechanicalObject", name="dofs", position=position, showObject=True, showObjectScale=8, drawMode=2, showColor=[1., 1., 1., 1.])
    target.createObject("UncoupledConstraintCorrection")
    return target


@SofaPrefab
class Trunk(SofaObject):
    """ This prefab is implementing a soft robot inspired by the elephant's trunk.
        The robot is entirely soft and actuated with 8 cables.

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
            trunk = Trunk(root)

            ## Direct access to the components
            trunk.displacements = [0., 0., 0., 0., 5., 0., 0., 0.]
    """

    def __init__(self, parentNode, youngModulus=450, poissonRatio=0.45, totalMass=0.042, inverseMode=False):

        self.inverseMode = inverseMode
        self.node = parentNode.createChild('Trunk')

        self.node.createObject('MeshVTKLoader', name='loader', filename=path+'trunk.vtk')
        self.node.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
        self.node.createObject('TetrahedronSetTopologyModifier')
        self.node.createObject('TetrahedronSetTopologyAlgorithms')
        self.node.createObject('TetrahedronSetGeometryAlgorithms')

        self.node.createObject('MechanicalObject', name='dofs', template='Vec3d', showIndices='false', showIndicesScale='4e-5')
        self.node.createObject('UniformMass', totalMass=totalMass)
        self.node.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=poissonRatio,  youngModulus=youngModulus)

        self.__addCables()

    def __addCables(self):
        length1 = 10.
        length2 = 2.
        lengthTrunk = 195.

        pullPoint = [[0., length1, 0.], [-length1, 0., 0.], [0., -length1, 0.], [length1, 0., 0.]]
        direction = Vec3(0., length2-length1, lengthTrunk)
        direction.normalize()

        nbCables = 4
        for i in range(0, nbCables):
            theta = 1.57*i
            q = Quat(0., 0., sin(theta/2.), cos(theta/2.))

            position = [[0., 0., 0.]]*20
            for k in range(0, 20, 2):
                v = Vec3(direction[0], direction[1]*17.5*(k/2)+length1, direction[2]*17.5*(k/2)+21)
                position[k] = v.rotateFromQuat(q)
                v = Vec3(direction[0], direction[1]*17.5*(k/2)+length1, direction[2]*17.5*(k/2)+27)
                position[k+1] = v.rotateFromQuat(q)

            cableL = self.node.createChild('cableL'+str(i))
            cableL.createObject('MechanicalObject', name='meca',
                                position=pullPoint[i]+[pos.toList() for pos in position])
            cableL.createObject('CableConstraint' if not self.inverseMode else 'CableActuator', template='Vec3d', name="cable",
                                hasPullPoint="0",
                                indices=range(0, 21),
                                maxPositiveDisp='70',
                                maxDispVariation="1",
                                minForce=0)
            cableL.createObject('BarycentricMapping', name='mapping',  mapForces='false', mapMasses='false')

        for i in range(0, nbCables):
            theta = 1.57*i
            q = Quat(0., 0., sin(theta/2.), cos(theta/2.))

            position = [[0., 0., 0.]]*10
            for k in range(0, 9, 2):
                v = Vec3(direction[0], direction[1]*17.5*(k/2)+length1, direction[2]*17.5*(k/2)+21)
                position[k] = v.rotateFromQuat(q)
                v = Vec3(direction[0], direction[1]*17.5*(k/2)+length1, direction[2]*17.5*(k/2)+27)
                position[k+1] = v.rotateFromQuat(q)

            cableS = self.node.createChild('cableS'+str(i))
            cableS.createObject('MechanicalObject', name='meca',
                                position=pullPoint[i]+[pos.toList() for pos in position])
            cableS.createObject('CableConstraint' if not self.inverseMode else 'CableActuator', template='Vec3d', name="cable",
                                hasPullPoint="0",
                                indices=range(0, 10),
                                maxPositiveDisp='40',
                                maxDispVariation="1",
                                minForce=0)
            cableS.createObject('BarycentricMapping', name='mapping',  mapForces='false', mapMasses='false')

    def addVisualModel(self, color=[1., 1., 1., 1.]):
        trunkVisu = self.node.createChild('VisualModel')
        trunkVisu.createObject('MeshSTLLoader', filename=path+"trunk.stl")
        trunkVisu.createObject('OglModel', template='ExtVec3d', color=color)
        trunkVisu.createObject('BarycentricMapping')

    def addCollisionModel(self, selfCollision=False):
        trunkColli = self.node.createChild('CollisionModel')
        for i in range(2):
            part = trunkColli.createChild("Part"+str(i+1))
            part.createObject('MeshSTLLoader', name="loader", filename=path+"trunk_colli"+str(i+1)+".stl")
            part.createObject('MeshTopology', src="@loader")
            part.createObject('MechanicalObject')
            part.createObject('TTriangleModel', group=1 if not selfCollision else i)
            part.createObject('TLineModel', group=1 if not selfCollision else i)
            part.createObject('TPointModel', group=1 if not selfCollision else i)
            part.createObject('BarycentricMapping')

    def fixExtremity(self):
        self.node.createObject('BoxROI', name='boxROI', box=[[-20, -20, 0], [20, 20, 20]], drawBoxes=False)
        self.node.createObject('PartialFixedConstraint', fixedDirections="1 1 1", indices="@boxROI.indices")

    def addEffectors(self, target, position=[0., 0., 195.]):
        effectors = self.node.createChild("Effectors")
        effectors.createObject("MechanicalObject", position=position)
        effectors.createObject("PositionEffector", indices=range(len(position)), effectorGoal=target)
        effectors.createObject("BarycentricMapping", mapForces=False, mapMasses=False)


def createScene(rootNode):

    rootNode.createObject("RequiredPlugin", name="SoftRobots")
    rootNode.createObject("RequiredPlugin", name="SofaSparseSolver")
    rootNode.createObject("RequiredPlugin", name="SofaPreconditioner")
    rootNode.createObject("RequiredPlugin", name="SofaPython")
    AnimationManager(rootNode)
    rootNode.createObject("VisualStyle", displayFlags="showBehavior")
    rootNode.gravity = [0., -9810., 0.]

    rootNode.createObject("FreeMotionAnimationLoop")
    # For direct resolution, i.e direct control of the cable displacement
    # rootNode.createObject("GenericConstraintSolver", maxIterations=100, tolerance=1e-5)
    # For inverse resolution, i.e control of effectors position
    rootNode.createObject("QPInverseProblemSolver", epsilon=1e-1)

    simulation = rootNode.createChild("Simulation")

    simulation.createObject('EulerImplicitSolver', name='odesolver', firstOrder="0", rayleighMass="0.1", rayleighStiffness="0.1")
    simulation.createObject('ShewchukPCGLinearSolver', name='linearSolver', iterations='500', tolerance='1.0e-18', preconditioners="precond")
    simulation.createObject('SparseLDLSolver', name='precond')
    simulation.createObject('GenericConstraintCorrection', solverName="precond")

    trunk = Trunk(simulation, inverseMode=True)
    trunk.addVisualModel(color=[1., 1., 1., 0.8])
    trunk.fixExtremity()
    target = effectorTarget(rootNode)
    trunk.addEffectors(target=target.dofs.getData("position").getLinkPath(), position=[[0., 0., 195]])

    # Use this in direct mode as an example of animation ############
    # def cableanimation(target, factor):
    #     target.cable.value = factor*20
    #
    # animate(cableanimation, {"target": trunk.cableL0}, duration=2, )
    #################################################################
