import Sofa

class controller(Sofa.PythonScriptController):

    def initGraph(self, node):
            self.node = node

    def onKeyPressed(self,c):
            inputvalue = self.node.getObject('aCable').findData('value')

            if (c == "+"):
               force = inputvalue.value[0][0] + 1000.
               inputvalue.value = [force]

            elif (c == "-"):
               force = inputvalue.value[0][0] - 1000.
               if(force < 0):
		           force = 0
               inputvalue.value = [force]
