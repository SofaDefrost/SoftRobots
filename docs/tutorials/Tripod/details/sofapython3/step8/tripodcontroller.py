import Sofa
import Sofa.Core
from splib3.numerics import RigidDof, Quat
from splib3.animation import animate
from splib3.constants import Key


def setupanimation(actuators, step, angularstep, factor):
    """This functions is called repeatidely in an animation.
       It moves the actuators by translating & rotating them according to the factor
       value.
    """
    for actuator in actuators:
        rigid = RigidDof(actuator.dofs)
        rigid.setPosition( rigid.rest_position + rigid.forward * step * factor )
        actuator.ServoMotor.angle = angularstep * factor


class GoalController(Sofa.Core.Controller):
    """This controller moves the goal position when the inverse control is activated
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "GoalController"
        self.activated = False
        self.time = 0
        self.dy = 0.1
        goalNode = args[1]
        self.mo = goalNode.goalMO
        self.dt = goalNode.getRoot().dt.value

    def onKeyPressed(self, key):
        if key == Key.I:
            self.activated = True

    def onAnimateBeginEvent(self, e):

        if self.activated:
            self.time = self.time+self.dt

        if self.time >= 1:
            self.time = 0;
            self.dy = -self.dy

        pos = [self.mo.position[0][0], self.mo.position[0][1], self.mo.position[0][2]]
        pos[1] += self.dy
        self.mo.position = [[pos[0], pos[1], pos[2], 0, 0, 0, 1]]


class DirectController(Sofa.Core.Controller):
    """This controller has two role:
       - if user press up/left/right/down/plus/minus the servo motor angle
         is changed.
       - if user press A an animation is started to move the motor to the physical position
         they are occupying in the real robot.
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "TripodController"
        self.stepsize = 0.1
        self.actuators = args[1]
        self.serialportctrl = args[2]

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.A and self.serialportctrl.state == "init":
            self.serialportctrl.state = "no-comm"
            animate(setupanimation, {"actuators": self.actuators, "step": 35.0, "angularstep": -1.4965}, duration=0.2)

        # Inclusion of the keystroke to start data sending = establishing communication ('comm')
        if key == Key.B and self.serialportctrl.state == "no-comm":
            self.serialportctrl.state = "comm"


class InverseController(Sofa.Core.Controller):
    """This controller has two role:
       - if user press up/left/right/down/plus/minus the servo motor angle
         is changed.
       - if user press A an animation is started to move the motor to the physical position
         they are occupying in the real robot.
    """

    def __init__(self, *args, serialport=None, servomotors=None, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "InverseController"
        self.nodeGoal = args[1]
        self.nodeEffector = args[2]
        self.nodeActuators = args[3]
        self.nodeDofRigid = args[4]
        self.nodeTripod = args[5]
        self.serialport = args[6]
        self.serialport.packetOut = [150, 150, 150]
        self.state = "init"
        self.actuators = servomotors
        self.activate = False

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.I:
            self.activate = 1
            self.nodeActuators.activated = bool(self.activate)
            self.nodeActuators.init()

    def onAnimateBeginEvent(self,e):

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
