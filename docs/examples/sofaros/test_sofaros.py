# coding: utf8
import sofaros
import numpy
  
def send(data):
        return numpy.asarray(data.value[0], dtype=numpy.float32)
        
def recv(data, datafield):
        t = data.tolist()
        datafield.value = [t[0]+1.0, t[1], t[2]]
        
def createScene(rootNode):
        sofaros.init("SofaNode")
        rootNode.createObject("EulerImplicitSolver")
        rootNode.createObject("CGLinearSolver")

        s=rootNode.createChild("simulated")
        s.createObject("MechanicalObject", name="sender", position=[0.0,0.0,0.0])
        s.sender.showObject=True
        s.sender.showObjectScale=1.0
        
        a=rootNode.createChild("animated")
        a.createObject("MechanicalObject", name="receiver", position=[0.0,0.0,0.0])
        a.receiver.showObject=True
        a.receiver.showObjectScale=1.0
        
        sofaros.RosSender(rootNode, "/simulation/sender/position", 
                          s.sender.findData("position"),  sofaros.numpy_msg(sofaros.Floats), send)
                          
        ## Entre le sender et le receiver...il y un midpoint ros.                
                          
        sofaros.RosReceiver(rootNode, "/animation/receiver/position", 
                            a.receiver.findData("position"), sofaros.numpy_msg(sofaros.Floats), recv)
        
        return rootNode
