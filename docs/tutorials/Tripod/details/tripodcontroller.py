import Sofa
from splib.numerics import RigidDof
from splib.animation import animate
from splib.constants import Key
from stlib.scene import Scene, Interaction
from tripod import Tripod


def setupanimation(actuators, step, angularstep, minRadius, maxRadius, factor):
    """This function is called repeatidely in an animation.
       It moves the actuators by translating & rotating them according to the factor
       value.
    """
    for actuator in actuators:
        rigid = RigidDof( actuator.node.ServoMotor.BaseFrame.dofs )
        rigid.setPosition( rigid.rest_position + rigid.forward * (maxRadius-minRadius) * factor )
        actuator.servomotor.angle = angularstep * factor


class MyController(Sofa.PythonScriptController):
    """This controller has two roles:
       - if the user presses up/left/right/down/plus/minus, the servomotor angle
         is changed.
       - if thr user presses A, an animation is started to move the servomotor to the initial position
         of the real robot.
    """

    def __init__(self, node, actuators):
        self.stepsize = 0.1
        self.actuators = actuators

    def onKeyPressed(self, key):
        self.initTripod(key)
        self.animateTripod(key)

    def initTripod(self, key):
        if key == Key.A:
            animate(setupanimation, {"actuators": self.actuators, "step": 35.0, 
                                     "angularstep": -0.60,
                                     "minRadius" : 10,
                                     "maxRadius" : 40
                                     }, duration=0.2)

    def animateTripod(self, key):
        if key == Key.uparrow:
            self.actuators[0].servomotor.angle += self.stepsize
        elif key == Key.downarrow:
            self.actuators[0].servomotor.angle -= self.stepsize

        if key == Key.leftarrow:
            self.actuators[1].servomotor.angle += self.stepsize
        elif key == Key.rightarrow:
            self.actuators[1].servomotor.angle -= self.stepsize

        if key == Key.plus:
            self.actuators[2].servomotor.angle += self.stepsize
        elif key == Key.minus:
            self.actuators[2].servomotor.angle -= self.stepsize


def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0])
    scene.VisualStyle.displayFlags = "showBehavior"

    scene.createObject("MeshSTLLoader", name="loader", filename="data/mesh/blueprint.stl")
    scene.createObject("OglModel", src="@loader")

    model = scene.createChild("Model")
    tripod = Tripod(model)

    MyController(rootNode, tripod.actuatedarms)

    Interaction(rootNode, targets=[tripod.ActuatedArm0,
                                   tripod.ActuatedArm1,
                                   tripod.ActuatedArm2])
