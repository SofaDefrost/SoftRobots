# -*- coding: utf-8 -*-

def CableActuation(node, position, value, pullPoint = [], valueType="displacement"):
        '''
        Adds: Child node with cable actuation

        Components added:
            MechanicalObject, CableConstaint and BarycentricMapping

        Parameters:
            position: cable path into the volume structure
            value: value to apply
            pullPoint: position from where the cable is pulled (if not specified the pullPoint will be considered on the structure) 
            valueType: type of the parameter value (displacement of force), default is displacement.
            
        Example:
        .. sourcecode:: python
            def createScene(rootNode)
                # Volume structure of the deformable robot
                model = rootNode.createChild("model")
                ...
                CableActuation(model, "0 0 0  0 0 10  0 0 20", 0)
        '''
        
        cable = node.createChild("cable")
        cable.createObject("MechanicalObject", position = position)
        if pullPoint not empty:
          cable.createObject("CableConstraint", indices=range(0,len(position)/3), valueType=valueType, value=value, pullPoint=pullPoint, hasPullPoint="1") 
        else:
          cable.createObject("CableConstraint", indices=range(0,len(position)/3), valueType=valueType, value=value, hasPullPoint="0")
        cable.createObject("BarycentricMapping", mapForces="false", mapMasses="false")
        
	
    
