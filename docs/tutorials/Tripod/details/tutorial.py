# -*- coding: utf-8 -*-
from stlib.scene import Scene as stScene, ContactHeader
from splib.objectmodel import setData, setTreeData

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
    if "plugins" not in kwargs:
        kwargs["plugins"] = []
        
    kwargs["plugins"].append("SofaSparseSolver")
        
    scene = stScene(parent, **kwargs)
    setData(scene, dt=0.025)
    setData(scene.VisualStyle, displayFlags = "showBehavior")

    scene.createObject("MeshSTLLoader", name="loader", filename="data/mesh/blueprint.stl")
    scene.createObject("OglModel", src="@loader")
    
    Modelling(scene)
    Simulation(scene)
    return parent

def addContact(self, alarmDistance=5, contactDistance=2, frictionCoef=0.0):
    ContactHeader(self,  alarmDistance, contactDistance, frictionCoef)

