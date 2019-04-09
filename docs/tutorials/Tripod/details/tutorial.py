# -*- coding: utf-8 -*-
from stlib.scene import Scene as stScene, ContactHeader
from splib.objectmodel import setData, setTreeData
from splib.loaders import getLoadingLocation
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

        for k,v in enumerate(data):
                visit(root, setMatchingData)

def Modelling(parent):
    """Create an empty node for modelling"""
    modeling = parent.createChild("Modelling")
    return modeling

def Simulation(parent):
    """Create an empty node for simulation"""
    simulation = parent.createChild("Simulation")
    simulation.createObject("EulerImplicitSolver")
    simulation.createObject("CGLinearSolver", iterations=120, tolerance=1e-20, threshold=1e-20)
    return simulation

def Scene(parent, **kwargs):
    import os
    if "plugins" not in kwargs:
        kwargs["plugins"] = []
        
    kwargs["plugins"].append("SofaSparseSolver")
        
    scene = stScene(parent, **kwargs)
    setData(scene, dt=0.025)
    setData(scene.VisualStyle, displayFlags = "showBehavior")

    scene.createObject("MeshSTLLoader", name="loader", filename=getLoadingLocation("data/mesh/blueprint.stl",__file__))
    scene.createObject("OglModel", src="@loader")

    scene.createObject("AddResourceRepository", path=os.path.abspath(os.path.dirname(__file__)))
        
    Modelling(scene)
    Simulation(scene)
    return parent

def addContact(self, alarmDistance=5, contactDistance=2, frictionCoef=0.0):
    ContactHeader(self,  alarmDistance, contactDistance, frictionCoef)

