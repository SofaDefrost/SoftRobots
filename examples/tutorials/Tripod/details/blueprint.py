# -*- coding: utf-8 -*-
"""
Step 3:
Make re-usable element out of a blueprint
"""
import Sofa


def Blueprint(name="Blueprint"):
    # Graphic modelling of the legends associated to the servomotors
    self = Sofa.Core.Node(name)
    self.addObject('MeshSTLLoader', name='loader', filename='data/mesh/blueprint.stl')
    self.addObject('OglModel', name='renderer', src='@loader')

    return self
