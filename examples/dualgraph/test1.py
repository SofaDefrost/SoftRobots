# -*- coding: utf-8 -*
from stlib.scene import MainHeader
from stlib.solver import DefaultSolver
from stlib.physics.rigid import Cube
from stlib.physics.constraints import FixedBox
from stlib.physics.deformable import ElasticMaterialObject
from os.path import commonprefix
from graphviz import Digraph, Graph
import Sofa

def addSubPointToRigid(c, p):
        s = c.createChild("subpoints")
        s.createObject("MechanicalObject", name="dofs", position=p)        
        s.createObject("RigidMapping", globalToLocalCoords=True)

def showDofs(d):
        d.showObject=True
        d.showObjectScale=0.2
        d.drawMode=2
        
def getAnimationLoop(node):
        loops = ["DefaultAnimationLoop","FreeMotionAnimationLoop"]
        def filterLoop(node, loops):
                for obj in node.getObjects():
                        print("O ", obj.getCategories())
                        c=obj.getCategories()
                        
                        if "BaseConstraint" in c:
                                loops.remove("DefaultAnimationLoop")
                        
                        if "BaseContactManager" in c:
                                print("RESPONSE: ", c.response)
                                loops.remove("DefaultAnimationLoop")
                         
                for child in node.getChildren():             
                        filterLoop(child, loops)
                        
        filterLoop(node, loops)
        return loops
       
# DefaultAnimationLoop
## EulerImplicit
### Solver

## EulerExplicit
### Solver  

# FreeMotionAnimationLoop 

colormap={
        "MechanicalState" : "aquamarine4",
        "Node" : "azure3",
        "Data" : "yellow", 
        "Loader" : "darkslategray1", 
        "CollisionModel": "lightpink",
        "Mapping" : "cyan4",
        "ForceField" : "violet",
        "InteractionForceField" : "orange"
        }

showlist = ["Node", "ForceField", "InteractionForceField", "Mapping", "MechanicalState"]
showhidden = True

def showGraph(gr):
        groups = {}
        nodes = {}
        dot = Digraph(comment='The Round Table', 
                      node_attr={'shape': 'record'})
        dot.attr(rankdir='LR')  
        dot.node_attr.update(fillcolor='red', style='filled')
        
        for k,g in gr["nodes"].items():
                if isinstance(g, Sofa.Node):
                        category = "Node"
                        name = g.getLinkPath()
                elif isinstance(g, Sofa.Data):
                        category = "Data"
                        name = g.getLinkPath()
                else:
                        category = g.getCategories()[-1]
                        name = g.name
                color = "white"
                
                if category in colormap:
                        color = colormap[category]
                else:
                        name+=" ["+category+"]"
                
                style = "invis"
                if showhidden or category in showlist:
                        nodes[g.getLinkPath()] = dot.node(g.getLinkPath(), name, fillcolor=color)
                #node.attr(fontsize='10')
        
        for k,g in gr["groups"].items():
                groups[k] = dot.subgraph(name="cluster_"+k)

                with groups[k] as kgraph:
                   for j in g:
                        if (isinstance(j, Sofa.BaseObject) or isinstance(j, Sofa.Data)) and j.getLinkPath() in nodes:
                                kgraph.edge(k, j.getLinkPath(), "", style="invis")
                        elif j.getLinkPath() in nodes:
                                dot.edge(k, j.getLinkPath(), "child")
                                
        for e in gr["edges"]:
                if e[2] == "context":
                        pass
                        #with groups[e[3]] as g:
                        #       g.edge(e[0], e[1], e[2])       
                elif e[0] in nodes and e[1] in nodes:
                        print("EDGES..")
                        dot.edge(e[0], e[1], e[2])       
                elif showhidden:
                        dot.edge(e[0], e[1], e[2]) 
        dot.render('test-output/round-table.gv', view=True)
        dot.view()

def dumpLinks(n, graph, spaces=""):
        spaces += "    " 
        print(spaces+"Scanning: "+ n.getLinkPath())
        graph["groups"][n.getLinkPath()] = []
        graph["nodes"][n.getLinkPath()] = n
                
        for c in n.getChildren():
                dumpLinks(c, graph, spaces=spaces+"    ") 
                graph["groups"][n.getLinkPath()].append(c)
                graph["nodes"][c.getLinkPath()] = c
                    
        for o in n.getObjects(): 
                graph["groups"][n.getLinkPath()].append(o)
                spaces2=spaces+"    "                             
                for l in o.getListOfLinks():
                        linkvalue = l.getValueString()
                        if l.getLinkedData() != None: 
                                if(l.name in ["input", "topology", "inputs"]):
                                        graph["edges"].append((l.getLinkedData().getLinkPath(), o.getLinkPath(), l.name, None)) 
                                else:
                                        graph["edges"].append((o.getLinkPath(), l.getLinkedData().getLinkPath(), l.name, None)) 
                                graph["groups"][n.getLinkPath()].append(l.getLinkedData())
                                graph["nodes"][l.getLinkedData().getLinkPath()] = l.getLinkedData()
                        elif l.getLinkedBase() != None:
                                if(l.name in ["input", "topology", "inputs"]):
                                        graph["edges"].append((l.getLinkedBase().getLinkPath(), o.getLinkPath(), l.name, None)) 
                                elif (l.name=="context"):
                                        graph["edges"].append((o.getLinkPath(), l.getLinkedBase().getLinkPath(), l.name, l.getLinkedBase().getLinkPath())) 
                                else:
                                        graph["edges"].append((o.getLinkPath(), l.getLinkedBase().getLinkPath(), l.name, None)) 
                                        
                graph["nodes"][o.getLinkPath()] = o
                
def generateSimulationPlan(target, source):
        for node in source.getChildren():
                interactionForceFields = node.interactionForceField.split()
                for iffpath in interactionForceFields:
                        ### [1:] is to remove the ugly @
                        iff = node.getObject(iffpath[1:])
                        object1 = node.getObject(iff.object1[1:])
                        object2 = node.getObject(iff.object2[1:])
                        object1Path = object1.getPathName()
                        object2Path = object2.getPathName()
                        #prefix = commonprefix(object1Path, object2Path)
        
def createScene(rootNode):
        MainHeader(rootNode)
 
        model = rootNode.createChild("Modelling")       
        p = [[2.0,2.0,2.0]]
        
        model.createObject("DefaultContactManager")
        
        ##### CUBE 
        c = Cube(model, name="cube", isAStaticObject=True, translation=[3.0,1.0,1.0])
        addSubPointToRigid(c, p)      
        showDofs(c.subpoints.dofs)

        ##### ELASTIC         
        e = ElasticMaterialObject(model, name="liver",
                          surfaceMeshFileName="mesh/liver.obj", 
                          volumeMeshFileName="mesh/liver.msh",
                          translation=[0.3,-2.2,2.0], youngModulus=10000)
        e.addSubPoint(p)
        showDofs(e)
        
        f = FixedBox(e, atPositions=[-5,-2,-2,-4,3,3])
        f.BoxROI.drawBoxes = True
        f.BoxROI.drawSize = 1
        
        print(": "+str(c.subpoints.getLinkPath()))
        
        ##### LA CONTRAINTE
        constraints = model.createChild("constraints")
        constraints.createObject("StiffSpringForceField", name="ff", 
                                 template="Vec3",
                                 object1=c.subpoints.getLinkPath(),             
                                 object2=e.subpoints.dofs.getLinkPath(),
                                 spring=[0, 0, 1000.0, 10.0, 0.0])         
        
        loops = getAnimationLoop(model)
        print("AvailableLoops ", loops)
        #AttachedPoints(e, c)         
        simulation = rootNode.createChild("Simulation")
        simulation.createObject("DefaultAnimationLoop")
        #simulation.createObject("SparseLDLSolver")
        simulation.createObject("CGLinearSolver")
        
        simulation.createObject("EulerImplicitSolver")
        generateSimulationPlan(simulation, model)        
        #simulation.createChild("constraintsMap")
        simulation.addChild(model.liver)
        simulation.addChild(model.cube)
        simulation.addChild(model.constraints)
        
        #model.constraints.addChild(model.liver)                
        #model.constraints.addChild(model.cube)                
        
        g = {"groups":{}, "nodes":{}, "edges":[]}
        #model.init()
        dumpLinks(model, g)
        showGraph(g)
