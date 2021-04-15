"""
In our framework, we handle the actuation and contact by defining specific constraints with Lagrange
multipliers on the boundary conditions of the deformable models. Different types of actuators are proposed (e.g cable and pneumatic actuators).

Contents
********
.. autosummary::
    :toctree: _autosummary

    CableConstraint
    SurfacePressureConstraint
    UnilateralPlaneConstraint


Related paper
**************

.. image:: https://team.inria.fr/defrost/files/2016/09/tadr_a_1395362_uf0001_oc.jpg
   :width: 100px
   :alt: Software toolkit for modeling, simulation and control of soft robots
   :align: left
   :target: https://hal.inria.fr/hal-01649355/document


| "`Software toolkit for modeling, simulation and control of soft robots`_",
| E. Coevoet, T. Morales-Bieze, F. Largilliere, Z. Zhang, M. Thieffry, et al.
| Advanced Robotics (2017)
|

.. _Software toolkit for modeling, simulation and control of soft robots: https://hal.inria.fr/hal-01649355/document

"""

__all__=["CableConstraint","SurfacePressureConstraint","UnilateralPlaneConstraint"]
