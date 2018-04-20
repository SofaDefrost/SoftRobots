from splib.objectmodel import SofaPrefab, SofaObject
from splib.numerics import getOrientedBoxFromTransform

@SofaPrefab
class FixingBox(SofaObject):
    """Fix a set of 'dofs' according to a translation & orientation"""
    def __init__(self, parent, target, name="FixingBox", 
                 translation=[0.0,0.0,0.0], eulerRotation=[0.0,0.0,0.0], scale=[1.0,1.0,1.0]):
        ob = getOrientedBoxFromTransform(translation, eulerRotation, scale)    

        self.node = parent.createChild(name)        
        self.node.createObject("BoxROI", orientedBox=ob, 
                                name="boxroi", position=target.dofs.getData("rest_position"), drawBoxes=True)
        
        c = self.node.createChild("Constraint")
        target.addChild(c)        
        c.createObject('RestShapeSpringsForceField',
                                               points=self.node.boxroi.getData("indices"), 
                                               stiffness='1e12')
        