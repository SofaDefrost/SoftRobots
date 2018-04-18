def createScene(rootNode):
    print("ROOT")
    
    child1 = rootNode.createChild("Child1")
    s = child1.createChild("SubChild1")
    s.createObject("OglModel", name="test")
    
    rootNode.createChild("Child2")
        
    #print("A :"+str(rootNode.test))
    print("A :"+str(rootNode.Child1))
    print("A :"+str(rootNode.Child2))
    print("A :"+str(rootNode.Child1.SubChild1.name))
    print("A :"+str(rootNode.Child1.SubChild1.test))

