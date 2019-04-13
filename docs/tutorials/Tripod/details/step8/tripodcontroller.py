import Sofa
from splib.numerics import RigidDof, Quat
from splib.animation import animate
from splib.constants import Key


def setupanimation(actuators, step, angularstep, factor):
    """This functions is called repeatidely in an animation.
       It moves the actuators by translating & rotating them according to the factor
       value.
    """
    for actuator in actuators:
        rigid = RigidDof(actuator.dofs)
        rigid.translate(rigid.forward * step * factor)
        actuator.ServoMotor.angle += angularstep * factor


class GoalController(Sofa.PythonScriptController):
    """This controller moves the goal position when the inverse control is activated
    """

    def __init__(self, goalNode):
        self.name = "GoalController"
        self.activated = False
        self.time = 0
        self.dy = 0.1
        self.mo = goalNode.goalMO

    def onKeyPressed(self, key):
        if key == Key.I:
            self.activated = True

    def onBeginAnimationStep(self, dt):

        if self.activated:
            self.time = self.time+dt

        if self.time >= 1:
            self.time = 0;
            self.dy = -self.dy

        pos = [self.mo.position[0][0], self.mo.position[0][1], self.mo.position[0][2]]
        pos[1] += self.dy
        self.mo.position = [[pos[0], pos[1], pos[2], 0, 0, 0, 1]]


class DirectController(Sofa.PythonScriptController):
    """This controller has two role:
       - if user press up/left/right/down/plus/minus the servo motor angle
         is changed.
       - if user press A an animation is started to move the motor to the physical position
         they are occupying in the real robot.
    """

    def __init__(self, node, actuators, serialportctrl):
        self.name = "TripodController"
        self.stepsize = 0.1
        self.actuators = actuators
        self.serialportctrl = serialportctrl

    def onKeyPressed(self, key):
        if key == Key.A and self.serialportctrl.state == "init":
            self.serialportctrl.state = "no-comm"
            animate(setupanimation, {"actuators": self.actuators, "step": 3.0, "angularstep": -0.14}, duration=0.2)

        # Inclusion of the keystroke to start data sending = establishing communication ('comm')
        if key == Key.B and self.serialportctrl.state == "no-comm":
            self.serialportctrl.state = "comm"


class InverseController(Sofa.PythonScriptController):
    """This controller has two role:
       - if user press up/left/right/down/plus/minus the servo motor angle
         is changed.
       - if user press A an animation is started to move the motor to the physical position
         they are occupying in the real robot.
    """

    def __init__(self, node, nodeGoal, nodeEffector, nodeActuators, nodeDofRigid, nodeTripod, serialport=None, servomotors=None):
        self.name = "InverseController"
        self.nodeGoal = nodeGoal
        self.nodeEffector = nodeEffector
        self.nodeActuators = nodeActuators
        self.nodeDofRigid = nodeDofRigid
        self.nodeTripod = nodeTripod
        self.serialport = serialport
        self.serialport.packetOut = [150, 150, 150]
        self.state = "init"
        self.actuators = servomotors
        self.activate = False

    def onKeyPressed(self, key):
        if key == Key.I:
            self.activate = 1

    def onBeginAnimationStep(self,dt):
        self.nodeActuators.activated = bool(self.activate)
        self.nodeActuators.init()

    def onEndAnimationStep(self,dt):

        if self.state == "init":
            return

        if self.state == "no-comm":
            return

        if(self.nodeActuators.activated):
            W_R_Dof = [0]*4;
            W_R_Ref = [0]*4;
            Ndir = [[ ] ]*3;
            Angles = [0]*3;
            for i in range(0,4):
                W_R_Dof[i] = self.nodeDofRigid.dofs.position[0][i+3]
                W_R_Ref[i] = self.nodeTripod.ActuatedArm0.ServoArm.dofs.position[0][i+3]

            [Ndir[0], Angles[0]] = Quat.product(Quat(W_R_Ref).getInverse(), Quat(W_R_Dof)).getAxisAngle()

            for i in range(0, 4):
                W_R_Dof[i] = self.nodeDofRigid.dofs.position[1][i+3]
                W_R_Ref[i] = self.nodeTripod.ActuatedArm1.ServoArm.dofs.position[0][i+3]

            [Ndir[1], Angles[1]] = Quat.product(Quat(W_R_Ref).getInverse(), Quat(W_R_Dof)).getAxisAngle()

            for i in range(0, 4):
                W_R_Dof[i] = self.nodeDofRigid.dofs.position[2][i+3]
                W_R_Ref[i] = self.nodeTripod.ActuatedArm2.ServoArm.dofs.position[0][i+3]

            [Ndir[2], Angles[2]] = Quat.product(Quat(W_R_Ref).getInverse(), Quat(W_R_Dof)).getAxisAngle()

            # conversion to degree (centred on 150)
            # envoie des infos aux servoMoteurs
            AnglesDeg = [0]*3;
            for i in range(3):
                if(Ndir[i][0]>0):
                    AnglesDeg[i] = Angles[i]*180/3.14159265359 + 90;
                else:
                    AnglesDeg[i] = -Angles[i]*180/3.14159265359 + 90;

                if AnglesDeg[i] < 60:
                    AnglesDeg[i] = 60
                if AnglesDeg[i] > 180:
                    AnglesDeg[i] = 180

            # The controller board of the real robot receives `AnglesDeg` values
            if(self.serialport):
                self.serialport.packetOut = [int(AnglesDeg[0]+0), int(AnglesDeg[2]-0), int(AnglesDeg[1])]
