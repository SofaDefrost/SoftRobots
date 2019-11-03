#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core

class FingerController(Sofa.Core.Controller):
    """
    This Controller simply takes the cableActuator's value
    and increases / decreases it depending on the pressed key ('+' or '-')
    """
    def __init__(self, *a, **kw):
        """
        In the ctor, we want to first call the constructor for the parent class (trampoline)
        We then store the node we want to retrieve the actuator from in the class
        (Sofa.Core.Base.getContext() could also have been used here instead,
        or a link between aCableActuator and the controller could have been used too)
        """
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.addData(name="displacement", type="double", value=kw.get("displacement"), default=0)
        return
    
    def onKeypressedEvent(self,e):
        """
        Events methods are named after the actual event names (Event::GetClassName() in C++),
        with a prepended "on" prefix. Thus this Event is the KeypressedEvent class in C++
        The onXXXEvent method takes a dictionary as a parameter, containing the useful
        values stored in the event class, e.g. here, the pressed key
        """

        print("keyPressed "+str(e))
        inputvalue = self.node.aCableActuator.value
        
        displacement = 0
        if (e["key"] == "+"):
            displacement = inputvalue.value[0] + 1.0
        elif (e["key"] == "-"):
            displacement = inputvalue.value[0] - 1.0
            if(displacement < 0):
                displacement = 0
                
        inputvalue.value = [displacement]
        return
