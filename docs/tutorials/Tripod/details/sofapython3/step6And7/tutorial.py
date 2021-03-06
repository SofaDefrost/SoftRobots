# -*- coding: utf-8 -*-
from stlib3.scene import Scene as stScene, ContactHeader
from splib3.objectmodel import setData, setTreeData
from splib3.loaders import getLoadingLocation
import functools
import Sofa


def dumpPosition(fields, filename):
    import json
    data = {}
    for field in fields:
        v = field.value
        if isinstance(v, list) and isinstance(v[0], list):
            v = v[0]
        data[field.getLinkPath()] = v
    with open(filename, "w+t") as file:
        json.dump(data, file)


def visit(target, fct):
        if isinstance(target, Sofa.Node):
                for child in target.getChildren():
                        visit(child, fct)
                for obj in target.getObjects():
                        fct(obj)
        fct(target)


def loadPosition(root, filename):
    import json

    with open(filename, "rt") as file:
        data = json.load(file)

        def setMatchingData(target):
            for field in target.getListOfDataFields():
                if field.getLinkPath() in data:
                    field.value = data[field.getLinkPath()]

        for k, v in enumerate(data):
            visit(root, setMatchingData)


def Modelling(parent):
    """add an empty node for modelling"""
    modeling = parent.addChild("Modelling")
    return modeling


def Simulation(parent):
    """add an empty node for simulation"""
    simulation = parent.addChild("Simulation")
    simulation.addObject("EulerImplicitSolver")
    simulation.addObject("CGLinearSolver", iterations=250, tolerance=1e-20, threshold=1e-20)
    return simulation


def Scene(parent, **kwargs):
    import os
    if "plugins" not in kwargs:
        kwargs["plugins"] = []

    kwargs["plugins"].append("SofaSparseSolver")

    scene = stScene(parent, **kwargs)
    setData(scene, dt=0.01)
    setData(scene, gravity=[0., -9810., 0.])
    setData(scene.VisualStyle, displayFlags="showForceFields")

    Modelling(scene)
    Simulation(scene)
    parent.addObject("FreeMotionAnimationLoop")
    parent.addObject("GenericConstraintSolver", maxIterations=250, tolerance=1e-20)

    ctx = scene.Config
    ctx.addObject("MeshSTLLoader", name="loader", filename=getLoadingLocation("../../data/mesh/blueprint.stl", __file__))
    ctx.addObject("OglModel", src="@loader")
    ctx.addObject("AddResourceRepository", path=os.path.abspath(os.path.dirname(__file__)))

    return parent


def addContact(self, alarmDistance=5, contactDistance=2, frictionCoef=0.0):
    ContactHeader(self,  alarmDistance, contactDistance, frictionCoef)
