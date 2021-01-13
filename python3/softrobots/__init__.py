# -*- coding: utf-8 -*-
"""
The SoftRobots plugin for SOFA: Templates Library Documentation
===============================================================

Utility functions and scene templates for the real-time simulation framework `SOFA <https://www.sofa-framework.org/>`_
and the `SoftRobots <https://project.inria.fr/softrobot/>`_ plugin.

The library can be used with scenes written in python and `PSL <https://github.com/sofa-framework/sofa/tree/master/applications/plugins/PSL>`_.

Example:
********

.. sourcecode:: python

    from stlib3.scene import MainHeader
    from stlib3.physics.rigid import Cube, Floor
    from stlib3.physics.deformable import ElasticMaterialObject

    from softrobots.actuators import PullingCable
    from softrobots.sensors import StringSensor

    def createScene(rootNode):
        MainHeader(rootNode)
        DefaultSolver(rootNode)

        Cube(rootNode, translation=[5.0,0.0,0.0])
        Floor(rootNode, translation=[0.0,-1.0,0.0])

       target = ElasticMaterialObject(volumeMeshFileName="mesh/liver.msh",
                                       totalMass=0.5,
                                       attachedTo=node)

        PullingCable(target)
        StringSensor(target)

Contents of the library
**********************

.. autosummary::
    :toctree: _autosummary

    softrobots.actuators
    softrobots.sensors
    softrobots.parts
    softrobots.inverse


Indices and tables
******************

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

"""

__all__=["actuators","parts", "sensors", "inverse"]
