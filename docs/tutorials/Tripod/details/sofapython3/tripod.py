import Sofa

from splib3.numerics import sin, cos, radians
from stlib3.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
from stlib3.physics.collision import CollisionMesh
from stlib3.physics.mixedmaterial import Rigidify
from stlib3.components import addOrientedBoxRoi
from splib3.numerics import vec3
from splib3.numerics.quat import Quat
from tutorial import *


def ElasticBody(parent):
    body = parent.addChild("ElasticBody")

    elasticMaterialObject = body.addChild(ElasticMaterialObject(volumeMeshFileName="../data/mesh/tripod_mid.gidmsh",
                                                                 translation=[0.0, 30, 0.0], rotation=[90, 0, 0],
                                                                 youngModulus=800, poissonRatio=0.45, totalMass=0.032))

    visual = body.addChild("Visual")
    visual.addObject("MeshSTLLoader", name="loader", filename="../data/mesh/tripod_mid.stl")
    visual.addObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                        rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.addObject("BarycentricMapping",
                        input=body.ElasticMaterialObject.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    return elasticMaterialObject


class Tripod(Sofa.Prefab):

    properties = [
        {'name':'name',                 'type':'string', 'help':'Node name',                   'default':'Tripod'},
        {'name':'radius',               'type':'int',    'help':'Rotation',                    'default':60},
        {'name':'numMotors',            'type':'int',    'help':'Translation',                 'default':3},
        {'name':'angleShift',           'type':'float', 'help':'Translation',                 'default':180.0}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):

        self.elasticMaterialObject = ElasticBody(self)
        self.rigidifiedstruct = None

        dist = self.radius.value
        numstep = self.numMotors.value
        self.actuatedarms = []
        for i in range(0, numstep):
            name = "ActuatedArm"+str(i)
            translation, eulerRotation = self.__getTransform(i, numstep, self.angleShift.value, self.radius.value, dist)
            arm = self.addChild(ActuatedArm(name=name,translation=translation, rotation=eulerRotation))
            self.actuatedarms.append(arm)
            # Add limits to angle that correspond to limits on real robot
            arm.ServoMotor.minAngle = -2.0225
            arm.ServoMotor.maxAngle = -0.0255

        self.__attachToActuatedArms(self.radius.value, self.numMotors.value, self.angleShift.value)

    def __getTransform(self, index, numstep, angleShift, radius, dist):
        fi = float(index)
        fnumstep = float(numstep)
        angle = fi*360/fnumstep
        angle2 = fi*360/fnumstep+angleShift
        eulerRotation = [0, angle, 0]
        translation = [dist*sin(radians(angle2)), -1.35, dist*cos(radians(angle2))]

        return translation, eulerRotation

    def addCollision(self):
        CollisionMesh(self.ElasticBody.ElasticMaterialObject,
                        surfaceMeshFileName="../data/mesh/tripod_low.stl", name="CollisionModel",
                        translation=[0.0, 30, 0.0], rotation=[90, 0, 0], collisionGroup=1)

    def __attachToActuatedArms(self, radius=60, numMotors=3, angleShift=180.0):
        deformableObject = self.elasticMaterialObject
        self.ElasticBody.init()
        dist = radius
        numstep = numMotors
        groupIndices = []
        frames = []
        for i in range(0, numstep):
            translation, eulerRotation = self.__getTransform(i, numstep, angleShift, radius, dist)

            box = addOrientedBoxRoi(self, position=[list(i) for i in deformableObject.dofs.rest_position.value], name="BoxROI"+str(i),
                                    translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                                    eulerRotation=eulerRotation, scale=[45, 15, 30])

            box.drawBoxes = False
            box.init()
            groupIndices.append([ind for ind in box.indices.value])
            frames.append(vec3.vadd(translation, [0.0, 25.0, 0.0]) + list(Quat.createFromEuler([0, float(i)*360/float(numstep), 0], inDegree=True)))

        effectorPos = [0, 30, 0]
        o = deformableObject.addObject('SphereROI', name='roi', template='Rigid3',
                                                    centers=effectorPos, radii=[7.5], drawSphere=True)
        o.init()
        groupIndices.append(list(o.indices.value))

        frames.append([effectorPos[0], effectorPos[1],
                       effectorPos[2], 0, 0, 0, 1])

        # Rigidify the deformable part at extremity to attach arms
        rigidifiedstruct = Rigidify(self, deformableObject, groupIndices=groupIndices, frames=frames, name="RigidifiedStructure")

        # Use this to activate some rendering on the rigidified object ######################################
        # setData(rigidifiedstruct.RigidParts.dofs, showObject=True, showObjectScale=10, drawMode=2)
        # setData(rigidifiedstruct.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=1,
        #         drawMode=1, showColor=[1., 1., 0., 1.])
        # setData(rigidifiedstruct.DeformableParts.dofs, showObject=True, showObjectScale=1, drawMode=2)
        #####################################################################################################

        # Attach arms
        rigidParts = rigidifiedstruct.RigidParts
        freecenter = rigidifiedstruct.addChild('FreeCenter')
        freecenter.addObject('MechanicalObject', name="dofs", template="Rigid3", position=[0,0,0,0,0,0,1])
        freecenter.addChild(rigidParts)
        for i in range(0, numstep):
            self.actuatedarms[i].ServoMotor.Articulation.ServoWheel.addChild(rigidParts)
        rigidParts.addObject('SubsetMultiMapping', input=[self.actuatedarms[0].ServoMotor.Articulation.ServoWheel.getLinkPath(),
                                                          self.actuatedarms[1].ServoMotor.Articulation.ServoWheel.getLinkPath(),
                                                          self.actuatedarms[2].ServoMotor.Articulation.ServoWheel.getLinkPath(),
                                                          freecenter.getLinkPath()],
                                                   output='@./', indexPairs=[0,1,1,1,2,1,3,0])
        rigidifiedstruct.removeChild(rigidParts)


def createScene(rootNode):
    from stlib3.scene import Scene

    scene = Scene(rootNode, gravity=[0., -9810., 0.],dt=0.01, iterative=False, plugins=["SofaSparseSolver", 'SofaDeformable', 'SofaEngine', 'SofaGeneralRigid', 'SofaMiscMapping', 'SofaRigid'])
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('GenericConstraintSolver', maxIterations=50, tolerance=1e-5)
    scene.Simulation.addObject('GenericConstraintCorrection')

    scene.VisualStyle.displayFlags = "showBehavior"

    tripod = scene.Modelling.addChild(Tripod())

    scene.Simulation.addChild(tripod.RigidifiedStructure)
    scene.Simulation.addObject('MechanicalMatrixMapper',
                                 name="mmm",
                                 template='Vec3,Rigid3',
                                 object1="@RigidifiedStructure/DeformableParts/dofs",
                                 object2="@RigidifiedStructure/FreeCenter/dofs",
                                 nodeToParse="@RigidifiedStructure/DeformableParts/ElasticMaterialObject")

    motors = scene.Simulation.addChild("Motors")
    for i in range(3):
        motors.addChild(tripod.getChild('ActuatedArm'+str(i)))
        scene.Simulation.addObject('MechanicalMatrixMapper',
                                     name="mmm"+str(i),
                                     template='Vec1,Vec3',
                                     object1="@Motors/ActuatedArm"+str(i)+"/ServoMotor/Articulation/dofs",
                                     object2="@RigidifiedStructure/DeformableParts/dofs",
                                     skipJ2tKJ2=True,
                                     nodeToParse="@RigidifiedStructure/DeformableParts/ElasticMaterialObject")
