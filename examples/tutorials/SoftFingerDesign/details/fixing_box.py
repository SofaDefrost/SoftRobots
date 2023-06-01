from splib3.objectmodel import SofaPrefab, SofaObject
from splib3.numerics import getOrientedBoxFromTransform


class FixingBox(SofaPrefab):
    """Fix a set of 'dofs' according to a translation & orientation"""

    def __init__(self, parent, target, name='FixingBox',
                 translation=[0.0, 0.0, 0.0], eulerRotation=[0.0, 0.0, 0.0], scale=[1.0, 1.0, 1.0]):

        ob = getOrientedBoxFromTransform(translation=translation, eulerRotation=eulerRotation, scale=scale)

        self.node = parent.addChild(name)
        self.node.addObject('BoxROI',
                               orientedBox=ob,
                               name='BoxROI',
                               position=target.dofs.getData('rest_position').getLinkPath(),
                               drawBoxes=False)

        c = self.node.addChild('Constraint')
        target.addChild(c)

        c.addObject('RestShapeSpringsForceField',
                       points=self.node.BoxROI.getData('indices').getLinkPath(),
                       stiffness=1e12)
