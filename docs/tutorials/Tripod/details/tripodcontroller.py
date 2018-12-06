import Sofa
from splib.numerics import RigidDof
from splib.animation import animate
from splib.constants import Key

def setupanimation(actuators, step, angularstep, rayonMin, rayonMax, factor):
    """This functions is called repeatidely in an animation.
       It moves the actuators by translating & rotating them according to the factor
       value.
    """
    for actuator in actuators:
            rigid = RigidDof( actuator.dofs )
            rigid.setPosition( rigid.getRestPosition() + rigid.forward * (rayonMax-rayonMin) * factor )
            actuator.ServoMotor.angle += angularstep * factor

class MyController(Sofa.PythonScriptController):
    """This controller has two role:
       - if user press up/left/right/down/plus/minus the servo motor angle
         is changed.
       - if user press A an animation is started to move the motor to the physical position
         they are occupying in the real robot.
    """
    def __init__(self, node, actuators):
        self.name = "TripodController"
        self.stepsize = 0.1
        self.actuators = actuators

    def onKeyPressed(self, key):
        if key == Key.uparrow:
            self.actuators[0].ServoMotor.angle += self.stepsize
        elif key == Key.downarrow:
            self.actuators[0].ServoMotor.angle -= self.stepsize

        if key == Key.leftarrow:
            self.actuators[1].ServoMotor.angle += self.stepsize
        elif key == Key.rightarrow:
            self.actuators[1].ServoMotor.angle -= self.stepsize

        if key == Key.plus:
            self.actuators[2].ServoMotor.angle += self.stepsize
        elif key == Key.minus:
            self.actuators[2].ServoMotor.angle -= self.stepsize

        if key == Key.A:
            animate(setupanimation,{"actuators" : self.actuators, "step" : 3.0,
                                    "angularstep" : -0.14, "rayonMax" : 65, "rayonMin" : 25}, duration=0.2)

        for actuator in self.actuators:
            if(actuator.ServoMotor.angle>-0.0225):
                actuator.ServoMotor.angle = -0.0255
            if(actuator.ServoMotor.angle<-2.0225):
                actuator.ServoMotor.angle = -2.0225
