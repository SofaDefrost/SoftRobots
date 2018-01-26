import Sofa

class controller(Sofa.PythonScriptController):

	def initGraph(self, node):
		self.node = node

	def onKeyPressed(self,c):

		if(c == '=' or c == '+'):
			inputvalue = self.node.getChild('cavity').getObject('surfaceConstraint').findData('value')
			displacement = inputvalue.value[0][0] + 5
			inputvalue.value = str(displacement)

		elif(c == '-'):
			inputvalue = self.node.getChild('cavity').getObject('surfaceConstraint').findData('value')
			displacement = inputvalue.value[0][0] - 5
			if(displacement < 0):
				displacement = 0
			inputvalue.value = str(displacement)
