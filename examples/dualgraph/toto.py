# -*- coding: utf-8 -*
from stlib.scene import MainHeader
from stlib.solver import DefaultSolver
from stlib.physics.rigid import Cube
from stlib.physics.deformable import ElasticMaterialObject

def addSubPointToRigid(c, p):
        s = c.createChild("subpoints")
        s.createObject("MechanicalObject", name="dofs", position=p)        
        s.createObject("RigidMapping", globalToLocalCoords=True)

def showDofs(d):
        d.showObject=True
        d.showObjectScale=0.2
        d.drawMode=2

def move(c, src, dst):
        print("TOTO" +str(c))
        src.removeChild(c)
        #dst.addChild(c)
        #src.addChild(c)
        
                
def generateSimulationPlan(rootNode):
        pass 
        
def createScene(rootNode):
        MainHeader(rootNode)
 
        model = rootNode.createChild("Modelling")       
        p = [[2.0,2.0,2.0]]
        
        ##### CUBE 
        c = Cube(model, name="cube", isAStaticObject=True, translation=[3.0, 1.0, 1.0])
        addSubPointToRigid(c, p)      
        showDofs(c.subpoints.dofs)

        ##### ELASTIC         
        e = ElasticMaterialObject(model, name="liver",
                          surfaceMeshFileName="mesh/liver.obj", 
                          volumeMeshFileName="mesh/liver.msh",
                          translation=[0.3,-2.2,2.0])
        e.addSubPoint(p)
        showDofs(e)
        
        ##### LA CONTRAINTE
        constraints = model.createChild("constraints")
        constraints.createObject("StiffSpringForceField", name="ff", 
                                 template="Vec3",
                                 object1=c.subpoints.getLinkPath(),             
                                 object2=e.subpoints.dofs.getLinkPath(),
                                 spring=[0, 0, 10000.0, 1000.0, 0.0])         
        
        #AttachedPoints(e, c)         
        simulation = rootNode.createChild("Simulation")
        #simulation.createObject("SparseLDLSolver")
        simulation.createObject("CGLinearSolver")
        simulation.createObject("EulerImplicitSolver")

        #simulation.createChild("constraintsMap")
        simulation.addChild(model.constraints)
        #simulation.constraints.addChild(model.cube)                
        #simulation.constraints.addChild(model.liver)                

