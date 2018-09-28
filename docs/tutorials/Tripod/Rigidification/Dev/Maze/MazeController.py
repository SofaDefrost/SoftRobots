import Sofa
from rigidification import *
from math import sin,cos
import os
path = os.path.dirname(os.path.abspath(__file__))+'/../details/data/mesh/'
path2 = os.path.dirname(os.path.abspath(__file__))+'/'

from numpy import *
from math import sqrt

#moi
def moveMazeToPosition(maze, pathToTetras, q):
    """This functions is called repeatidely in an animation.
       Forces the center of the maze to be positionned where the center of the silicone is 
    """
    indices_pts_center=[228,238,239,250,251,260,261,262,265,280,281,294,295,302,303,316,317,324,327,338,339,352,353,364]


    average_center=[0.0,0.0,0.0]
    for pos in range(len(indices_pts_center)):
        average_center[0]=average_center[0]+pathToTetras[indices_pts_center[pos]][0]
        average_center[1]=average_center[1]+pathToTetras[indices_pts_center[pos]][1]
        average_center[2]=average_center[2]+pathToTetras[indices_pts_center[pos]][2]
    average_center[0]=average_center[0]/len(indices_pts_center)-30.0
    average_center[1]=average_center[1]/len(indices_pts_center)-25.0#+2.0
    average_center[2]=average_center[2]/len(indices_pts_center)

    #maze.dofs.position= average_center + maze.dofs.position[0][3:]
    #maze.dofs.position= average_center + [-0.707107,0.0,0.0,0.707107] #more stable
    pos = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    pos[0]=average_center[0]
    pos[1]=average_center[1]
    pos[2]=average_center[2]
    pos[3]=q[1]  #-0.707
    pos[4]=q[2]
    pos[5]=q[3]
    pos[6]=q[0]  #+0.707
    print pos
    maze.dofs.position=pos


def ShapeMatrix(indices_pts_center, pathToTetras):
    """Builds an Nx3 matrix of the points at the center of the silicone piece """
    B=zeros((len(indices_pts_center), 3))
    for N in range(len(indices_pts_center)):
        for i in range(3):
            B[N,i]=pathToTetras[indices_pts_center[N]][i]
    return B



# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    """ From a set of points, computes the rotation matrix and translation that transforms the points in A to the points in B 
        Code from here, very slightly altered: http://nghiaho.com/?page_id=671
    """
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)
    
    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    #H = transpose(AA) * BB
    H = dot(transpose(AA),BB)

    U, S, Vt = linalg.svd(H)
    print"ok"

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    print t

    return R

def matrixToQuaternion(R):
    """ get the quaternions from a rotation matrix R: NOT WORKING
    """
    tr = R[0][0] + R[1][1] + R[2][2]

    if tr > 0:
        S = sqrt(tr+1.0) * 2 # S=4*qw 
        qw = 0.25 * S
        qx = (R[2][1] - R[1][2]) / S
        qy = (R[0][2] - R[2][0]) / S 
        qz = (R[1][0] - R[0][1]) / S
    elif ((R[0][0] > R[1][1])and(R[0][0] > R[2][2])):
        S = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2 # S=4*qx 
        qw = (R[2][1] - R[1][2]) / S
        qx = 0.25 * S
        qy = (R[0][1] + R[1][0]) / S
        qz = (R[0][2] + R[2][0]) / S
    elif (R[1][1] > R[2][2]): 
        S = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2 # S=4*qy
        qw = (R[0][2] - R[2][0]) / S
        qx = (R[0][1] + R[1][0]) / S 
        qy = 0.25 * S
        qz = (R[1][2] + R[2][1]) / S 
    else:
        S = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2 # S=4*qz
        qw = (R[1][0] - R[0][1]) / S
        qx = (R[0][2] + R[2][0]) / S
        qy = (R[1][2] + R[2][1]) / S
        qz = 0.25 * S

    print qx, qy, qz, qw

    return [qx, qy, qz, qw]

def matrixToQuaternion2(matrix, isprecise=False):
    """ code here: https://github.com/matthew-brett/transforms3d/blob/master/original/transformations.py """
    M = array(matrix, copy=False)[:4, :4]
    if isprecise:
        q = empty((4, ))
        t = trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = linalg.eigh(K)
        q = V[[3, 0, 1, 2], argmax(w)]
    if q[0] < 0.0:
        negative(q, q)
    print q
    return q


    
def moveMazeTranslationAndRotation(maze, pathToTetras):
    """ This function is called at each timestep to recompute the current translation and rotation parameters of the maze, mirroring the one of the center of the silicone piece
    """

    indices_pts_center=[228,238,239,250,251,260,261,262,265,280,281,294,295,302,303,316,317,324,327,338,339,352,353,364]

    ## Code to build the translation and rotation matrices based on the points at the center of the silicone
    ## initiaal positions of the points at the center of the silicone piece
    A = matrix([[6.406,21.818,7.847],
            [2.146,21.737,10.077],
            [8.612,21.896,3.930],
            [3.066,21.770,7.070],
            [5.549,21.924,4.828],
            [-0.470,21.752,7.948],
            [6.299,21.946,1.552],
            [-2.906,21.644,10.023],
            [8.328,22.027,-0.847],
            [-3.924,21.653,6.983],
            [5.288,22.108,-1.969],
            [-7.104,21.609,7.857],
            [5.916,22.190,-5.155],
            [-6.223,21.687,4.699],
            [2.848,22.073,-4.538],
            [-7.051,21.850,1.548],
            [-0.288,22.146,-5.452],
            [-9.306,21.677,3.924],
            [1.911,22.304,-7.723],
            [-5.983,22.019,-1.947],
            [-3.401,22.079,-4.526],
            [-9.085,22.861,-0.852],
            [-2.569,22.272,-7.737],
            [-6.514,22.103,-5.157]])
    B = ShapeMatrix(indices_pts_center, pathToTetras)
    return rigid_transform_3D(A,B)



####

class MazeControl(Sofa.PythonScriptController):

    def __init__(self, controllerNode, name):
        # note: this member aliases the sofa component `name` data, so you can't
        # put anything in here (in this case, strings only)
        self.name=name

        self.currentNode = controllerNode
        self.time=0
        


    #moi
    def onBeginAnimationStep(self, dt):

        #pathToTetras=rootNode.robot.nodeFEM.tetras.position


        self.rootNode = self.currentNode.getRoot()
        self.simumaze = self.rootNode.getChild("SimuMaze")
        self.maze = self.simumaze.getChild("Maze")

        #self.simutripod = self.rootNode.getChild("SimuTripod")
        #self.robot = self.simutripod.getChild("robot")
        self.robot = self.rootNode.getChild("robot")

        self.rigid = self.robot.getChild("Rigid")
        self.rigidified = self.rigid.getChild("Rigidified")
        self.nodefem = self.rigidified.getChild("nodeFEM")

        self.pathToTetras = self.nodefem.tetras.position

        print self.maze.getObject("dofs").findData("position").value[0][1]

        #moveMazeToPosition(self.maze, self.pathToTetras)
        #self.maze.getObject("dofs ").findData("position").value[0][0]=0.0
        
        #self.maze.dofs.position= [0.0,22.0,0.0] + self.maze.dofs.position[0][3:]
       
        #moveMazeToPosition(self.maze, self.pathToTetras)
        R = moveMazeTranslationAndRotation(self.maze, self.pathToTetras)
        #q=matrixToQuaternion(R) #NOT WORKING
        q=matrixToQuaternion2(R)
        moveMazeToPosition(self.maze, self.pathToTetras, q)


        #test mvt: 
        #self.time = self.time + dt
        #self.maze.dofs.position= [0.0,2*self.time*22.0,0.0,-0.707107,0.0,0.0,0.707107]
        #moveMazeToPosition(self.maze,self.pathToTetras)

        self.maze.dofs.velocity= [0.0,0.0,0.0,0.0,0.0,0.0]
        self.maze.dofs.force= [0.0,0.0,0.0,0.0,0.0,0.0]
        #self.maze.getObject("dofs").findData("translation").value[0][1]=30.0
        #self.maze.getObject("dofs").findData("position").value[0][2]=0.0

        print self.maze.getObject("dofs").findData("position").value[0][1]
    ####
    #USELESS in the end, it was an attempt at correcting the position of the maze and its collision mesh once more at the end of the timestep
    # def onEndAnimationStep(self,dt):
    #     self.rootNode = self.currentNode.getRoot()
    #     self.simumaze = self.rootNode.getChild("SimuMaze")
    #     self.maze = self.simumaze.getChild("Maze")

    #     self.maze.dofs.position= [0.0,22.0,0.0] + self.maze.dofs.position[0][3:]

# 