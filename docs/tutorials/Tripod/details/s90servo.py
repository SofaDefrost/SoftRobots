import os
from splib.objectmodel import SofaObject, SofaPrefab, setData
from splib.numerics import RigidDofZero
from splib.animation import animate
from stlib.scene import Scene
import Sofa
dirPath = os.path.dirname(os.path.abspath(__file__))+'/'

def VisualBody(parent, position=RigidDofZero):    
    visual = parent.createChild("VisualModel")
    visual.createObject("MeshSTLLoader", name="loader", filename=dirPath+"data/mesh/SG90_servomotor.stl")
    visual.createObject("MeshTopology", src="@loader")
    visual.createObject("OglModel", color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
    visual.createObject("RigidMapping", index=0)
    return visual

#class DataEngine(Sofa.PythonScriptDataEngine):
#        def __init__(self, owner, position, rotation):
#                self.owner = owner 
#                self.name = "dataupdate"

#                self.addNewInput('position', datatype="Vec3d", value=position)       
#                self.addNewInput('rotation', datatype="Vec3d", value=rotation)       
#               self.rigiddof = self.addNewOutput("rigiddof", datatype="vector<double>", value=RigidDofZero)

 #       def update(self):   
 #               print("UPDATE")
                #self.angleInDegree = self.angleInput[0][0] / (2 * 3.14) * 360


@SofaPrefab
class ServoMotor(SofaObject):
    """A S90 servo motor

    This prefab is implementing a S90 servo motor.
    https://servodatabase.com/servo/towerpro/sg90

    The prefab ServoMotor is composed of:
    - a visual model
    - a mechanical model composed two rigids. One rigid is for the motor body
      while the other is implementing the servo rotating wheel.

    The prefab has the following parameters:
    - translation           to change default location of the servo (default [0.0,0.0,0.0])
    - rotation              to change default rotation of the servo (default [0.0,0.0,0.0,1])
    - scale                 to change default scale of the servo (default 1)
    - showServo             to control wether a visual model of the motor is added (default True)
    - showWheel             to control wether the rotation axis of the motor is displayed (default False)

    The prefab have the following property:
    - angle         use this to specify the angle of rotation of the servo motor
    - angleLimits   use this to set a min and max value for the servo angle rotation
    - position      use this to specify the position of the servo motor

    Example of use in a Sofa scene:

    def createScene(root):
        ...
        servo = ServoMotor(root)

        ## Direct access to the components
        servo.angle.value = 1.0
    """

    def __init__(self, parent,
                 translation=[0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0], 
                 scale=[1.0, 1.0, 1.0], showServo=True, showWheel=False):

        self.node = parent.createChild("ServoMotor")

         ## The inputs
        self.node.addNewData("angleIn", "S90Properties", "angle of rotation (in radians)", "float", 0)
        #self.node.addNewData("positionIn", "S90Properties", "position of the basis of the servo", "Vec3d", [0,0,0])
        #self.node.addNewData("rotationIn", "S90Properties", "position of the basis of the servo", "Vec3d", [0,0,0])

        # Two positions (rigid): first one for the servo body, second for the servo wheel
        position = self.node.createChild("BaseFrame")
        #e = DataEngine(position, 
        #               self.node.getData("positionIn").getLinkPath(), 
        #               self.node.getData("rotationIn").getLinkPath())
        
        position.createObject("MechanicalObject", name="dofs", 
                              template="Rigid3", 
                              #position=e.rigiddofs.getLinkPath(),
                              translation=translation, rotation=rotation,
                              #translation2=translation, rotation2=rotation, 
                              scale3d=scale)
        #position.createObject("UniformMass", totalMass=0.1)                      
                              

        # Angle of the wheel
        angle = position.createChild("ArticulatedJoint")
        angle.createObject("MechanicalObject", name="dofs", template="Vec1d", position=self.node.getData("angleIn").getLinkPath())

        # This component is used to constrain the angle to lie between a maximum and minimum value,
        # corresponding to the limit of the real servomotor
        angle.createObject("ArticulatedHierarchyContainer")
        angle.createObject('StopperConstraint', name="AngleLimits", index=0)
        angle.createObject("UncoupledConstraintCorrection")
        
        d = angle.createChild("WheelFrame")
        dd = d.createObject("MechanicalObject", name="slavedofs", template="Rigid3", position=RigidDofZero*2)
        d.createObject("ArticulatedSystemMapping", input1=angle.dofs.getLinkPath(), 
                                                   input2=position.dofs.getLinkPath(), output=dd.getLinkPath())

        articulationCenter = angle.createChild("ArticulationCenter")
        articulationCenter.createObject("ArticulationCenter", parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.], posOnChild=[0., 0., 0.])
        articulation = articulationCenter.createChild("Articulations")
        articulation.createObject("Articulation", translation=False, rotation=True, rotationAxis=[1, 0, 0], articulationIndex=0)

        s = self.node.addNewData("angleOut", "S90Properties", "angle of rotation (in degree)", "float", angle.dofs.getData("position").getLinkPath())

        # ServoBody and ServoWheel objects with visual
        servobody = VisualBody(self.BaseFrame)

        self.node.init()
        self.node.BaseFrame.dofs.translation = [0,0,0]
        self.node.BaseFrame.dofs.rotation = [0,0,0]

    def setAngleLimits(self, angleLimits):
        self.node.BaseFrame.ArticulatedJoint.AngleLimits.min = angleLimits[0]
        self.node.BaseFrame.ArticulatedJoint.AngleLimits.max = angleLimits[1]

    angleLimits = property(fset=setAngleLimits)
                
def createScene(rootNode):
    import math
    import Sofa
    from splib.constants import Key
    from splib.animation import animate
    
    def animation(self, factor):
        if self.doIncr:
            self.target.angleIn = math.sin(factor * math.pi/4)
            
            print("AngleInput           : ", self.target.angleIn)
            print("AngleOutput          : ", self.target.angleOut)                        

    class KeyBoardActions(Sofa.PythonScriptController):        
        def __init__(self, node, target):
                self.target = target
                self.doIncr = True
                self.animation = animate(animation, {"self" : self}, duration=5., mode="loop")
                
        def onKeyPressed(self, key):
            if key == Key.A:
                self.doIncr = not self.doIncr

    Scene(rootNode)

    rootNode.dt = 0.003
    rootNode.gravity = [0., -9810., 0.]
    rootNode.createObject("VisualStyle", displayFlags="showBehaviorModels")

    # Use these components on top of the scene to solve the constraint "StopperConstraint".
    #rootNode.createObject("FreeMotionAnimationLoop")
    rootNode.createObject("GenericConstraintSolver", maxIterations=1e3, tolerance=1e-5)

    simulation = rootNode.createChild("Simulation")
    simulation.createObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
    simulation.createObject("CGLinearSolver", name="precond")

    ServoMotor(simulation, showWheel=True)
    KeyBoardActions(simulation, simulation.ServoMotor)

    setData(simulation.ServoMotor.BaseFrame.dofs,  showObject=True, showObjectScale=10)
    setData(simulation.ServoMotor.BaseFrame.ArticulatedJoint.WheelFrame.slavedofs,  showObject=True, showObjectScale=10)

    return rootNode
