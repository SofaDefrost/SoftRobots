# -*- coding: utf-8 -*-
from stlib.scene import Scene
from stlib.loader import addLoader

def createScene(rootNode):
    rootNode.createObject("MechanicalObject")
    tetras = addLoader(rootNode, filename="data/mesh/tripod.gidmsh")

    triangles = rootNode.createChild("triangles")     
    container = triangles.createObject("TriangleSetTopologyContainer", name="container")
    triangles.createObject("MechanicalObject")
    
    triangles.createObject("TriangleSetTopologyModifier", name="modifier")    
    triangles.createObject("TriangleSetTopologyAlgorithms", name="topologyalgorithms")    
    triangles.createObject("TriangleSetGeometryAlgorithms", name="topologicalmapping")    
    triangles.createObject("Tetra2TriangleTopologicalMapping", 
                          name="topologicalmapping2", 
                          input=tetras.getLinkPath(), output=container.getLinkPath())

