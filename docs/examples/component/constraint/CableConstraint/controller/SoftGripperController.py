#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa

class controller(Sofa.PythonScriptController):

    def initGraph(self, node):
            self.node1 = node.getChild('finger1').getChild('actuator1')
            self.node2 = node.getChild('finger2').getChild('actuator2')
            self.node3 = node.getChild('finger3').getChild('actuator3')

    def onKeyPressed(self,c):
      
            inputvalue1 = self.node1.getObject('CableConstraint').findData('value')
            inputvalue2 = self.node2.getObject('CableConstraint').findData('value')
            inputvalue3 = self.node3.getObject('CableConstraint').findData('value')
            
            if (c == "+"):
               displacement1 = inputvalue1.value[0][0] + 1.
               inputvalue1.value = str(displacement1)
               
               displacement2 = inputvalue2.value[0][0] + 1.
               inputvalue2.value = str(displacement2)
               
               displacement3 = inputvalue3.value[0][0] + 1.
               inputvalue3.value = str(displacement3)
               
            elif (c == "-"):
               displacement1 = inputvalue1.value[0][0] - 1.
               if(displacement1 < 0):
		  displacement1 = 0
               inputvalue1.value = str(displacement1)
               
               displacement2 = inputvalue2.value[0][0] - 1.
               if(displacement2 < 0):
		  displacement2 = 0
               inputvalue2.value = str(displacement2)
               
               displacement3 = inputvalue3.value[0][0] - 1.
               if(displacement3 < 0):
		  displacement3 = 0
               inputvalue3.value = str(displacement3)
 




