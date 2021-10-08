from splib3.numerics import sin, cos, radians
from stlib3.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
import math
from stlib3.physics.mixedmaterial import Rigidify


def ElasticBody(parent):
    body = parent.addChild('ElasticBody')

    e = body.addChild(ElasticMaterialObject(body,
                              volumeMeshFileName='../../data/mesh/tripod_mid.gidmsh',
                              translation=[0.0, 30, 0.0], rotation=[90, 0, 0],
                              youngModulus=600, poissonRatio=0.45, totalMass=0.4))

    visual = e.addChild('Visual')
    visual.addObject('MeshSTLLoader', name='loader', filename='../../data/mesh/tripod_mid.stl')
    visual.addObject('MeshTopology', name='topology', src='@loader')
    visual.addObject('OglModel', name='renderer', src='@topology', color=[1.0, 1.0, 1.0, 0.5], rotation=[90, 0, 0], translation=[0, 30, 0])
    visual.addObject('BarycentricMapping')
    return body


# Let's define a Tripod prefab now, that we can later call in the addScene
# function
def Tripod(parent, name='Tripod', radius=60, numMotors=3, angleShift=180.0, effectorPos=None, use_orientation=True, goalNode=None):
    tripod = parent.addChild(name)

    # It is using the ElasticBody that was made in the previous step, and that
    # has also been included in the tripod.py script.
    body = ElasticBody(tripod)
    body.init()
    # The actuated arms are positionned around the silicone piece using a loop
    # structure
    dist = radius
    numstep = numMotors
    b = []
    arms = []
    angles = []
    frames = []
    for i in range(0, numstep):
        name = 'ActuatedArm'+str(i)
        fi = float(i)
        fnumstep = float(numstep)
        angle = fi*360/fnumstep
        angle2 = fi*360/fnumstep+angleShift
        eulerRotation = [0, angle, 0]
        angles.append([0, angle, 0])
        translation = [dist*sin(radians(angle2)),
                       -1.35,
                       dist*cos(radians(angle2))]

        frames.append([dist*sin(radians(angle2)),
                       -1.35,
                       dist*cos(radians(angle2)),
                       0, angle, 0])

        c = tripod.addChild(ActuatedArm(tripod, name=name,
                        translation=translation, eulerRotation=eulerRotation, showServoArm=False))

        c.addBox(body.ElasticMaterialObject.dofs.getData('rest_position'),
                 translation, eulerRotation)
        arms.append(c)
        b.append(list(c.Box.BoxROI.indices.value))

    if effectorPos is not None and len(effectorPos) == 3:
        o = body.ElasticMaterialObject.addObject('SphereROI', name='roi', template='Rigid3',
                                                    position=body.ElasticMaterialObject.dofs.getData('rest_position'),
                                                    centers=effectorPos, radii=[7.5], drawSphere=True)
        o.init()
        b.append(list(o.indices.value))

        frames.append([effectorPos[0], effectorPos[1],
                       effectorPos[2], 0, 0, 0])

    o = Rigidify(tripod,
                 body.ElasticMaterialObject,
                 name='RigidifiedStructure',
                 frames=frames,
                 groupIndices=b)

    actuators = o.RigidParts.addChild('actuators')

    actuator0 = actuators.addObject('SlidingActuator', name='SlidingActuator0', template='Rigid3', direction=[0, 0, 0, 1, 0, 0], indices=0,
                                        maxPositiveDisp=-0.5, maxNegativeDisp=1.5, maxDispVariation=0.02, initDisplacement=-1.47)
    actuator1 = actuators.addObject('SlidingActuator', name='SlidingActuator1', template='Rigid3', direction=[0, 0, 0, cos(4*math.pi/3), 0, sin(4*math.pi/3)], indices=1,
                                        maxPositiveDisp=-0.5, maxNegativeDisp=1.5, maxDispVariation=0.02, initDisplacement=-1.47)
    actuator2 = actuators.addObject('SlidingActuator', name='SlidingActuator2', template='Rigid3', direction=[0, 0, 0, cos(2*math.pi/3), 0, sin(2*math.pi/3)], indices=2,
                                        maxPositiveDisp=-0.5, maxNegativeDisp=1.5, maxDispVariation=0.02, initDisplacement=-1.47)

    if goalNode == None:
        Effector = actuators.addObject('PositionEffector', name='effector', template='Rigid3', useDirections=[1, 1, 1, 0, 0, 0],
                                          indices=3, effectorGoal=[10, 40, 0], limitShiftToTarget=True, maxShiftToTarget=5)
    elif use_orientation:
        Effector = actuators.addObject('PositionEffector', name='effector', template='Rigid3', useDirections=[0, 1, 0, 1, 0, 1],
                                          indices=3, effectorGoal=goalNode.goalMO.getLinkPath()+'.position', limitShiftToTarget=True, maxShiftToTarget=5)
    else:
        Effector = actuators.addObject('PositionEffector', name='effector', template='Rigid3', useDirections=[1, 1, 1, 0 ,0 ,0],
                                          indices=3, effectorGoal=goalNode.goalMO.getLinkPath()+'.position', limitShiftToTarget=True, maxShiftToTarget=5)

    actuators.activated = 0
    tripod.addObject('MechanicalMatrixMapper',
                        template='Vec3,Rigid3',
                        object1=o.DeformableParts.getLinkPath(),
                        object2=o.RigidParts.dofs.getLinkPath(),
                        nodeToParse=o.RigidParts.RigidifiedParticules.ElasticMaterialObject.getLinkPath())

    for i in range(0, numMotors):
        a = arms[i].ServoMotor.addChild('Attach')
        a.addObject('MechanicalObject', template='Rigid3', name='dofs',
                       showObject=True, showObjectScale=10,
                       position=[0.0, 25.0, 10.0, 0, 0, 0, 1])
        a.addObject('RigidRigidMapping')

        o.RigidParts.addObject('RestShapeSpringsForceField', name='rssff'+str(i),
                                  external_rest_shape=arms[i].ServoArm.dofs.getLinkPath(),
                                  points=[i], external_points=[0],
                                  angularStiffness=1e7, stiffness=1e10)

    for i in range(0, numMotors):
        arms[i].addObject('FixedConstraint')
        arms[i].ServoMotor.ServoWheel.addObject('FixedConstraint')

    tripod.removeChild(body)

    return tripod


def createScene(rootNode):
    from stlib3.scene import Scene

    scene = Scene(rootNode, gravity=[0., -9810., 0.],dt=0.025, plugins=["SofaSparseSolver","SoftRobots","SoftRobots.Inverse",'SofaBoundaryCondition', 'SofaDeformable', 'SofaEngine', 'SofaGeneralRigid', 'SofaGeneralAnimationLoop', 'SofaMiscMapping', 'SofaRigid'])
    scene.addMainHeader()
    scene.addObject('DefaultAnimationLoop')
    scene.addObject('DefaultVisualManagerLoop')

    scene.VisualStyle.displayFlags = "showBehavior"

    tripod = Tripod(scene.Modelling)

    scene.Simulation.addChild(tripod.RigidifiedStructure)
    motors = scene.Simulation.addChild("Motors")
    motors.addChild(tripod.ActuatedArm0)
    motors.addChild(tripod.ActuatedArm1)
    motors.addChild(tripod.ActuatedArm2)
