from splib.numerics import sin, cos, to_radians
from stlib.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
import math
from stlib.physics.mixedmaterial import Rigidify


def ElasticBody(parent):
    body = parent.createChild("ElasticBody")

    e = ElasticMaterialObject(body,
                              volumeMeshFileName="../data/mesh/tripod_mid.gidmsh",
                              translation=[0.0,30,0.0], rotation=[90,0,0],
                              youngModulus=600, poissonRatio=0.45, totalMass=0.4)

    visual = body.createChild("Visual")
    visual.createObject("MeshSTLLoader", name="loader", filename="../data/mesh/tripod_mid.stl")
    visual.createObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                        rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.createObject("BarycentricMapping",
                         input=e.dofs.getLinkPath(),
                         output=visual.renderer.getLinkPath())
    return body


# Let's define a Tripod prefab now, that we can later call in the createScene
# function
def Tripod(parent, name="Tripod", radius=60, numMotors=3, angleShift=180.0, effectorPos=None, use_orientation=True, goalNode=None):
    tripod = parent.createChild(name)

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
        name = "ActuatedArm"+str(i)
        fi = float(i)
        fnumstep = float(numstep)
        angle = fi*360/fnumstep
        angle2 = fi*360/fnumstep+angleShift
        eulerRotation = [0, angle, 0]
        angles.append([0, angle, 0])
        translation = [dist*sin(to_radians(angle2)),
                       -1.35,
                       dist*cos(to_radians(angle2))]

        frames.append([dist*sin(to_radians(angle2)),
                       -1.35,
                       dist*cos(to_radians(angle2)),
                       0, angle, 0])

        c = ActuatedArm(tripod, name=name,
                        translation=translation, eulerRotation=eulerRotation)

        c.addBox(body.ElasticMaterialObject.dofs.getData("rest_position"),
                 translation, eulerRotation)
        arms.append(c)
        b.append(map(lambda x: x[0], c.Box.BoxROI.indices))


    if len(effectorPos) == 3:
        o = body.ElasticMaterialObject.createObject("SphereROI", name="roi", template="Rigid3",
                    position=body.ElasticMaterialObject.dofs.getData("rest_position"),
                    centers=effectorPos, radii=[7.5], drawSphere=True)
        o.init()
        b.append(map(lambda x: x[0], o.indices))

        frames.append([effectorPos[0],effectorPos[1],effectorPos[2],0,0,0])

    o = Rigidify(tripod,
                body.ElasticMaterialObject,
                name="RigidifiedStructure",
                frames=frames,
                groupIndices=b)

    actuators = o.RigidParts.createChild('actuators')

    actuator0 = actuators.createObject('SlidingActuator', name="SlidingActuator0", template='Rigid3d', direction='0 0 0 1 0 0' , indices=0, maxForce='100000', minForce='-30000')
    actuator1 = actuators.createObject('SlidingActuator', name="SlidingActuator1", template='Rigid3d', direction='0 0 0 '+str(cos(4*math.pi/3))+' 0 '+str(sin(4*math.pi/3)) , indices=1, showDirection='1', showVisuScale='100', maxForce='100000', minForce='-30000')
    actuator2 = actuators.createObject('SlidingActuator', name="SlidingActuator2", template='Rigid3d', direction='0 0 0 '+str(cos(2*math.pi/3))+' 0 '+str(sin(2*math.pi/3)) , indices=2, showDirection='1',  showVisuScale='100', maxForce='100000', minForce='-30000')

    if goalNode==None:
        Effector = actuators.createObject('PositionEffector', name='effector', template='Rigid3d', useDirections='1 1 1 0 0 0', indices='3', effectorGoal="10 40 0", limitShiftToTarget=True, maxShiftToTarget=5 )
    elif use_orientation:
        Effector = actuators.createObject('PositionEffector', name='effector', template='Rigid3d', useDirections='0 1 0 1 0 1', indices='3', effectorGoal=goalNode.goalMO.getLinkPath()+".position", limitShiftToTarget=True, maxShiftToTarget=5)
    else:
        Effector = actuators.createObject('PositionEffector', name='effector', template='Rigid3d', useDirections='1 1 1 0 0 0', indices='3', effectorGoal= goalNode.goalMO.getLinkPath()+".position", limitShiftToTarget=True, maxShiftToTarget=5)

    actuators.activated = 0
    tripod.createObject('MechanicalMatrixMapper',
                        template='Vec3,Rigid3',
                        object1=o.DeformableParts.getLinkPath(),
                        object2=o.RigidParts.dofs.getLinkPath(),
                        nodeToParse = o.RigidParts.RigidifiedParticules.ElasticMaterialObject.getLinkPath())

    for i in range(0, numMotors):
        a = arms[i].ServoMotor.createChild("Attach")
        a.createObject("MechanicalObject", template="Rigid3d", name="dofs",
                       showObject=True, showObjectScale=10,
                       position=[0.0, 25.0, 10.0, 0, 0, 0, 1])
        a.createObject("RigidRigidMapping")

        o.RigidParts.createObject('RestShapeSpringsForceField',
                                  external_rest_shape=arms[i].ServoArm.dofs.getLinkPath(),
                                  points=[i], external_points=[0],
                                  angularStiffness=1e7, stiffness=1e10)

    for i in range(0, numMotors):
        arms[i].createObject("FixedConstraint")
        arms[i].ServoMotor.ServoWheel.createObject("FixedConstraint")

    tripod.removeChild(body)

    return tripod
