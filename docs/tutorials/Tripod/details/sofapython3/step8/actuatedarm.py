# -*- coding: utf-8 -*-
''' ActuatedArm for the tripod robot.

    This model is part of the SoftRobot toolkit available at:
        https://github.com/SofaDefrost/SoftRobots

    Available prefab:
        - ActuatedArm
        - ServoArm
'''
import Sofa
from splib3.numerics import vec3, Transform
from splib3.objectmodel import SofaPrefab
from stlib3.visuals import VisualModel
from stlib3.components import addOrientedBoxRoi
from splib3.scenegraph import *

from s90servo import ServoMotor


class ServoArm(Sofa.Prefab):
    '''ServoArm is a reusable sofa model of a servo arm for the S90 servo motor

       Parameters:
            parent:        node where the ServoArm will be attached
            mappingInput:  the rigid mechanical object that will control the orientation of the servo arm
            indexInput: (int) index of the rigid the ServoArm should be mapped to
    '''

    properties = [
        {'name':'name','type':'string', 'help':'Node name','default':'ServoArm'},
        {'name':'mappingInputLink','type':'string',  'help':'the rigid mechanical object that will control the orientation of the servo arm','default':''},
        {'name':'indexInput','type':'int',  'help':'index of the rigid the ServoArm should be mapped to','default':0}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.addObject('MechanicalObject',
                               name='dofs',
                               size=1,
                               template='Rigid3',
                               showObject=True,
                               showObjectScale=5)

    def setRigidMapping(self,path):

        self.addObject('RigidRigidMapping',name='mapping', input=path, index=self.indexInput.value)

    def addVisual(self):
        visual = self.addChild(VisualModel(visualMeshPath='../../data/mesh/SG90_servoarm.stl', translation=[0., 0., 0.], color=[1., 1., 1., 0.75]))
        visual.OglModel.writeZTransparent = True
        visual.addObject('RigidMapping', name='mapping')

class ActuatedArm(Sofa.Prefab):
    '''ActuatedArm is a reusable sofa model of a S90 servo motor and the tripod actuation arm.
           Parameters:
             - translation the position in space of the structure
             - eulerRotation the orientation of the structure
             - attachingToLink (MechanicalObject)    a rest shape forcefield will constraint the object
                                                 to follow arm position
             - showServoArm default is True
           Structure:
           Node : {
                name : 'ActuatedArm'
                MechanicalObject     // Rigid position of the motor
                ServoMotor           // The s90 servo motor with its actuated wheel
                ServoArm             // The actuation arm connected to ServoMotor.ServoWheel
            }
    '''
    properties = [
        {'name':'name',                'type':'string', 'help':'Node name',                   'default':'ActuatedArm'},
        {'name':'eulerRotation',       'type':'Vec3d',  'help':'Rotation',                    'default':[0.0, 0.0, 0.0]},
        {'name':'translation',         'type':'Vec3d',  'help':'Translation',                 'default':[0.0, 0.0, 0.0]},
        {'name':'scale',               'type':'Vec3d',  'help':'Scale 3d',                    'default':[1.0, 1.0, 1.0]},
        {'name':'attachingToLink',     'type':'string', 'help':'a rest shape forcefield will constraint the object to follow arm position','default':''},
        {'name':'showServoArm',        'type':'bool', 'help':'show arm', 'default':True}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):

        r = Transform(translation=list(self.translation.value), eulerRotation=list(self.eulerRotation.value))
        self.addObject("MechanicalObject", name="dofs", size=1, position=r.toSofaRepr(), template='Rigid3', showObject=True, showObjectScale=15)

        self.servomotor = self.addChild(ServoMotor(name="ServoMotor"))
        self.servomotor.addObject("RigidRigidMapping", name="mapping")
        self.servoarm = self.addChild(ServoArm(name="ServoArm"))
        self.servoarm.setRigidMapping(self.ServoMotor.ServoWheel.dofs.getLinkPath())
        if self.showServoArm.value :
            self.servoarm.addVisual()

        if self.attachingToLink :
            attachingTo = getFromRoot(self,self.attachingToLink)
            constraint = self.addConstraint(attachingTo, self.translation.value, self.eulerRotation.value)
            self.attachingTo.addObject('RestShapeSpringsForceField', name='rssff'+self.name,
                                     points=constraint.BoxROI.getData('indices'),
                                     external_rest_shape=constraint.dofs,
                                     stiffness='1e12')

    def addBox(self, position, translation, eulerRotation):
        constraint = self.addChild('Box')
        # template of box is Rigid3 because of its location
        o = addOrientedBoxRoi(constraint, position=[list(pos)+[0,0,0,1] for pos in position.value],
                              translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                              eulerRotation=eulerRotation, scale=[45, 15, 30])
        o.drawBoxes = False
        o.init()

    def addConstraint(self, deformableObject, translation, eulerRotation):
        constraint = self.addChild('Constraint')
        o = addOrientedBoxRoi(constraint, position=deformableObject.dofs.getData('rest_position'),
                              translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                              eulerRotation=eulerRotation, scale=[45, 15, 30])
        o.drawSize = 1
        o.drawBoxes = False

        constraint.addObject('TransformEngine', input_position='@BoxROI.pointsInROI',
                                translation=translation, rotation=eulerRotation, inverse=True)

        constraint.addObject('MechanicalObject', name='dofs',
                                template='Vec3', position='@TransformEngine.output_position',
                                showObject=True, showObjectScale=5.0)

        constraint.addObject('RigidMapping', name='mapping', input=self.ServoMotor.ServoWheel.dofs, output='@./')

        return constraint


def createScene(rootNode):
    from splib3.animation import animate
    from stlib3.scene import Scene
    import math

    scene = Scene(rootNode, plugins=['SofaConstraint', 'SofaGeneralRigid', 'SofaRigid', 'SofaBoundaryCondition', 'SofaOpenglVisual'])
    scene.addMainHeader()
    scene.addObject('DefaultVisualManagerLoop')
    scene.addObject('DefaultAnimationLoop')
    scene.VisualStyle.displayFlags = 'showBehavior'
    rootNode.dt = 0.003
    rootNode.gravity = [0., -9810., 0.]

    arm = rootNode.Simulation.addChild(ActuatedArm(name='ActuatedArm', translation=[0.0, 0.0, 0.0]))
    arm.addObject("FixedConstraint")

    def myanimate(target, factor):
        target.angle = math.cos(factor * 2 * math.pi)

    animate(myanimate, {'target': arm.ServoMotor}, duration=5, mode='loop')
