import Sofa.Core

class ControllerPressure(Sofa.Core.Controller):

	def __init__(self, *a, **kw):
		Sofa.Core.Controller.__init__(self, *a, **kw)
		self.node = kw["node"]
		return

	def onKeypressedEvent(self,e):
		inputvalue = self.node.cavity.surfaceConstraint.value

		pressure = 0
		if (e["key"] == Sofa.constants.key_plus):
			pressure = inputvalue.value[0] + 5.0
		elif (e["key"] == Sofa.constants.key_minus):
			pressure = inputvalue.value[0] - 5.0
			if(pressure < 0):
				pressure = 0

		inputvalue.value = [pressure]
		return
