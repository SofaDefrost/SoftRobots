import Sofa

class controller(Sofa.PythonScriptController):

    def initGraph(self, node):
            self.node = node

    def onKeyPressed(self,c):
            inputvalue = self.node.getObject('aCable').findData('value')

            if (c == "+"):
               displacement = inputvalue.value[0][0] + 1000.
               inputvalue.value = str(displacement)

            elif (c == "-"):
               displacement = inputvalue.value[0][0] - 1000.
               if(displacement < 0):
		  displacement = 0
               inputvalue.value = str(displacement)
