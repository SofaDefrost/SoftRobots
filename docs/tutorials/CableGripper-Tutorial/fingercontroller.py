# -*- coding: utf-8 -*-
import Sofa

class FingerCableController(Sofa.PythonScriptController):
    def initGraph(self, node):
        self.node = node

    def onKeyPressed(self,c):
        inputvalue = self.node.getObject('aCable').findData('value')

        if (c == "+"):
            inputvalue.value =  inputvalue.value[0][0] + 1.

        elif (c == "-"):
            displacement = inputvalue.value[0][0] - 1.
            if(displacement < 0):
                displacement = 0
            inputvalue.value = displacement



