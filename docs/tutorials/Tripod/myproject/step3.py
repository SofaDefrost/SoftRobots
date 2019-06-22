# -*- coding: utf-8 -*-
"""
Step 3:
In this step, the use of code blocks using functions is introduced, as reusable elements of the code.
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),"../details"))
from tutorial import *

# A prefab that implements an ElasticMaterialObject
from stlib.physics.deformable import ElasticMaterialObject

# A prefab to fix some part of an object at its rest position.
from fixingbox import FixingBox


# This function includes the whole mechanical model of the silicone piece, as written in the previous step, except that the prefab ElasticMaterialObject is used, instead of creating each component.
def ElasticBody(parent):
    """Create an object with an elastic deformation law"""
    body = parent.createChild("ElasticBody")

    # Prefab ElasticMaterialObject implementing the whole mechanical model of the silicone piece
    e = ElasticMaterialObject(body,
                              volumeMeshFileName="data/mesh/tripod_mid.gidmsh",
                              poissonRatio=0.45,
                              youngModulus=800,
                              totalMass=0.032,
                              rotation=[90, 0, 0])

    # Visual model
    visual = e.createChild("Visual")
    visual.createObject("MeshSTLLoader",
                        name="loader",
                        filename="data/mesh/tripod_mid.stl",
                        rotation=[90, 0, 0])
    visual.createObject("OglModel",
                        name="renderer",
                        src="@loader",
                        color=[1.0, 1.0, 1.0, 0.5])
    visual.createObject("BarycentricMapping",
                        input=e.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())
    return body

def createScene(rootNode):
    scene = Scene(rootNode, gravity=[0.0, -9810, 0.0])
    scene.dt = 0.025
    scene.VisualStyle.displayFlags = "showBehavior"

    scene.Config.createObject("MeshSTLLoader", name="loader", filename="data/mesh/blueprint.stl")
    scene.Config.createObject("OglModel", src="@loader")

    # Instanciating the prefab into the graph
    body = ElasticBody(scene.Modelling)

    # Instanciating the FixingBox prefab into the graph, constraining the mechanical object of the ElasticBody.
    fix = FixingBox(scene.Modelling,
                    body.ElasticMaterialObject,
                    eulerRotation=[0,0,0],
                    translation=[0.0, .0, 0.0],
                    scale=[30., 30., 30.])

    # Changing the property of the Box ROI so that the constraint area appears on screen.
    fix.BoxROI.drawBoxes = True
