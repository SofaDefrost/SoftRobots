import Sofa
import Sofa.Core

class ControllerForce(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        return

    def onKeypressedEvent(self,e):
        inputvalue = self.node.aCable.value

        force = 0
        if (e["key"] == Sofa.constants.key_plus):
            force = inputvalue.value[0] + 1.0
        elif (e["key"] == Sofa.constants.key_minus):
            force = inputvalue.value[0] - 1.0
            if(force < 0):
                force = 0

        inputvalue.value = [force]
        return
