import params
from math import sin, cos, sqrt


# Takes a 3d vector and return its norm
def norm(x):
    n = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2])
    return n


# Takes a 3d vector and normalize it
def normalize(x):
    n = norm(x)
    for i in range(0,3):
        x[i] = x[i]/n


# Multiplication of two quaternions (gives the composition of the two rotation)
def rotateQuat(q1,q2):

    c0 = q1[6]*q2[3] - q1[3]*q2[0] - q1[4]*q2[1] - q1[5]*q2[2]
    c1 = q1[6]*q2[0] + q1[3]*q2[3] - q1[4]*q2[2] + q1[5]*q2[1]
    c2 = q1[6]*q2[1] + q1[4]*q2[3] + q1[3]*q2[2] - q1[5]*q2[0]
    c3 = q1[5]*q2[3] - q1[6]*q2[2] + q1[3]*q2[1] + q1[4]*q2[0]

    q1 = [q1[0],q1[1],q1[2],c1,c2,c3,c0]

    return q1


# Rotate a vector using a quaternion v'=qvq(-1)
def rotate(v,q):

    c0 = ((1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))*v[0] + (2.0 * (q[0] * q[1] - q[2] * q[3])) * v[1] + (2.0 * (q[2] * q[0] + q[1] * q[3])) * v[2])
    c1 = ((2.0 * (q[0] * q[1] + q[2] * q[3]))*v[0] + (1.0 - 2.0 * (q[2] * q[2] + q[0] * q[0]))*v[1] + (2.0 * (q[1] * q[2] - q[0] * q[3]))*v[2])
    c2 = ((2.0 * (q[2] * q[0] - q[1] * q[3]))*v[0] + (2.0 * (q[1] * q[2] + q[0] * q[3]))*v[1] + (1.0 - 2.0 * (q[1] * q[1] + q[0] * q[0]))*v[2])

    v[0] = c0
    v[1] = c1
    v[2] = c2

    return v


def generateDOFTransform(position0, position1, DOF0, DOF1, dx, angle):

    DOFTransformNode=[[0, 0, 0, 0, 0, 0, 1]]*2;
    for i in range(3):
        DOFTransformNode[0][i] = position0[i] - DOF0[i];
        DOFTransformNode[1][i] = position1[i] - DOF1[i];

    direction = [position0[i]-position1[i] for i in range(3)]
    normalize(direction)
    direction = [direction[0] * dx, direction[1] * dx, direction[2] * dx]

    DOFTransformNode[0] = [DOFTransformNode[0][i]+direction[i] for i in range(3)] + DOFTransformNode[0][3:7]
    q = [0, 0, sin(angle/2), cos(angle/2)]
    DOFTransformNode[1] = rotateQuat(DOFTransformNode[1], q)

    return DOFTransformNode


# Add the first rib (different from the others because 5 points instead of 4)
def addFirstRib(length, height, angle):

    # Initialize position of nodes and edges list
    position = [[0,0,0]]*5
    edges = [[0, 2], [1, 2], [0, 3], [1, 3], [0, 4], [1, 4]]
    position[0] = [0, 0, 0]
    position[1] = [0, height*2, 0]
    position[2] = [length, height, 0]

    for k in [3,4]:  # Compute the four other beams positions by applying a rotation of 120 and 240 degrees
        v = [position[2][i] - position[0][i] for i in range(3)]
        theta = 2.0944*(k-2)
        q = [0., sin(theta/2.), 0., cos(theta/2.)]
        v = rotate(v,q)
        position[k] = [position[0][i] + v[i] for i in range(3)]

    # Computation of the transformation beam / nodes (to allow straight beams not aligned with x)
    numBeam = len(edges)
    DOF0TransformNode0 = [[0,0,0,0,0,0,1]]*numBeam
    DOF1TransformNode1 = [[0,0,0,0,0,0,1]]*numBeam

    for b in range(numBeam):
        position0=[0]*3;
        position1=[1]*3;

        for i in range(3):
            position0[i] = position[edges[b][0]][i]
            position1[i] = position[edges[b][1]][i]

        if b % 2==0:
            transform = generateDOFTransform(position0, position1, position0, position1, params.Arm.dist, -angle)
        else:
            transform = generateDOFTransform(position0, position1, position0, position1, params.Arm.dist, angle)

        DOF0TransformNode0[b] = transform[0]
        DOF1TransformNode1[b] = transform[1]

    # Rotate beams frame
    i = 1
    for k in range(2,6):
        theta = 2.0944*i
        if k % 2==1: i = i+1
        q=[0,sin(theta/2.),0,cos(theta/2.)]
        DOF0TransformNode0[k] = rotateQuat(DOF0TransformNode0[k],q)
        DOF1TransformNode1[k] = rotateQuat(DOF1TransformNode1[k],q)

    return [position, edges, DOF0TransformNode0, DOF1TransformNode1]


# Add rib
# position: vector of positions of the robots considering all the added ribs
# edges: list of all edges
# DOF0TransformNode0 and DOF1TransformNode1: list that links dofs to corresponding nodes (for BeamInterpolation component)
def addRib(length, height, angle, ribId, position, edges, DOF0TransformNode0, DOF1TransformNode1):

    # Initialize position of nodes and edges list
    if ribId == 0:
        [position, edges, DOF0TransformNode0, DOF1TransformNode1] = addFirstRib(length, height, angle)
        return [position, edges, DOF0TransformNode0, DOF1TransformNode1]

    position = position + [[0, 0, 0]] * 4
    edges = edges + [[(ribId - 1) * 4 + 1, ribId * 4 + 2],
                     [ribId * 4 + 1, ribId * 4 + 2],
                     [(ribId - 1) * 4 + 1, ribId * 4 + 3],
                     [ribId * 4 + 1, ribId * 4 + 3],
                     [(ribId - 1) * 4 + 1, ribId * 4 + 4],
                     [ribId * 4 + 1, ribId * 4 + 4]]

    fid = 5+(ribId-1)*4
    prevHeight = position[(ribId - 1) * 4 + 1][1]
    position[fid] = [0, height * 2 + prevHeight, 0]
    position[fid + 1] = [length, height + prevHeight, 0]

    i = fid
    for k in [0,0,1,2]:  # Compute the four other beams positions by applying a rotation of 120 and 240 degrees
        if k != 0:
            v = [position[fid+1][i] - position[fid][i] for i in range(3)]
            theta = 2.0944*k
            q = [0., sin(theta/2.), 0., cos(theta/2.)]
            v = rotate(v,q)
            position[i] = [position[fid][i] + v[i] for i in range(3)]
        i += 1

    # Computation of the transformation beam / nodes (to allow straight beams not aligned with x)
    DOF0TransformNode0 = DOF0TransformNode0 + [[0,0,0,0,0,0,1]]*6
    DOF1TransformNode1 = DOF1TransformNode1 + [[0,0,0,0,0,0,1]]*6

    numBeam = len(edges)

    for b in range(6):
        position0=[0]*3;
        position1=[1]*3;

        for i in range(3):
            position0[i] = position[edges[b + numBeam - 6][0]][i]
            position1[i] = position[edges[b + numBeam - 6][1]][i]

        if b % 2==0:
            transform = generateDOFTransform(position0, position1, position0, position1, params.Arm.dist, -angle)
        else:
            transform = generateDOFTransform(position0, position1, position0, position1, params.Arm.dist, angle)

        DOF0TransformNode0[b+numBeam-6] = transform[0]
        DOF1TransformNode1[b+numBeam-6] = transform[1]

    # Rotate beams frame
    i = 6*ribId
    for k in [0,0,1,1,2,2]:
        if k != 0:
            theta = 2.0944*k
            q = [0,sin(theta/2.),0,cos(theta/2.)]
            DOF0TransformNode0[i] = rotateQuat(DOF0TransformNode0[i],q)
            DOF1TransformNode1[i] = rotateQuat(DOF1TransformNode1[i],q)
        i += 1

    return [position, edges, DOF0TransformNode0, DOF1TransformNode1]


# Generation of the arm (based on params.py)
def generateRibs():
    position = []
    edges = []
    DOF0TransformNode0 = []
    DOF1TransformNode1 = []
    for s in range(params.Arm.nbSection):
        for i in range(params.Arm.nbRibs):
            ribId = params.Arm.nbRibs * s + i
            [position, edges, DOF0TransformNode0, DOF1TransformNode1] = addRib(params.Arm.beamLength,
                                                                               params.Arm.beamHeight[s],
                                                                               params.Arm.angle,
                                                                               ribId, position, edges,
                                                                               DOF0TransformNode0,
                                                                               DOF1TransformNode1)

    return [position, edges, DOF0TransformNode0, DOF1TransformNode1]


# Add a node with 3 CableActuator in the given arm node
def addCables(node, length, nbSection):

    cables = node.addChild('Cables')
    theta = 2.0944
    q = [0., sin(theta/2.), 0., cos(theta/2.)]
    size = params.Arm.nbRibs * nbSection

    positions = [[0,0,0],[0,0,0],[0,0,0]] * size
    p = [[params.Arm.tx, 0, 0]] * 3
    v = p[0]
    v = rotate(v,q); p[1] = [v[0],v[1],v[2]]
    v = rotate(v,q); p[2] = [v[0],v[1],v[2]]
    v = rotate(v,q)
    for i in range(size):
        positions[i*3+0] = [p[0][0], p[0][1], p[0][2]]
        positions[i*3+1] = [p[1][0], p[1][1], p[1][2]]
        positions[i*3+2] = [p[2][0], p[2][1], p[2][2]]

    cables.addObject('MechanicalObject', position=positions)

    pullPoint = [[0,0,0],[0,0,0],[0,0,0]]
    pullPoint[0] = [length + params.Arm.tx, 0, 0]
    v = pullPoint[0];
    v = rotate(v,q); pullPoint[1] = [v[0],v[1],v[2]]
    v = rotate(v,q); pullPoint[2] = [v[0],v[1],v[2]]
    v = rotate(v,q);

    if params.Simulation.inverse:
        for i in range(3):
            cables.addObject('CableActuator', name="cable"+str(i), minForce=0, indices=list(range(i,size*3,3)), pullPoint=pullPoint[i])
    else:
        for i in range(3):
            cables.addObject('CableConstraint', name="cable"+str(i), indices=list(range(i,size*3,3)), pullPoint=pullPoint[i], valueType="displacement")

    rigidIndexPerPoint = [list(range(4*i+2,(i+1)*4+1)) for i in range(size)]
    cables.addObject('RigidMapping', rigidIndexPerPoint=rigidIndexPerPoint, mapForces=False, mapMasses=False, applyRestPosition=True)

    return
