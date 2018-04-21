import Sofa
from splib.numerics import RigidDof
from splib.animation import animate
from splib.constants import Key

def setupanimation(actuators, step, angularstep, factor):
    """This functions is called repeatidely in an animation.
       It moves the actuators by translating & rotating them according to the factor
       value.
    """
    for actuator in actuators:
            rigid = RigidDof( actuator.dofs )
            rigid.translate( rigid.forward * step * factor )
            actuator.ServoMotor.angle += angularstep * factor

class MyController(Sofa.PythonScriptController):
    """This controller has two role:
       - if user press up/left/right/down/plus/minus the servo motor angle
         is changed.
       - if user press A an animation is started to move the motor to the physical position
         they are occupying in the real robot.
    """
    def __init__(self, node, actuators):
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
                                    "angularstep" : -0.14}, duration=0.2)
