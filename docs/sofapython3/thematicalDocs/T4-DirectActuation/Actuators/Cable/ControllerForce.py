import Sofa
import Sofa.Core
import Sofa.constants.Key as Key


class ControllerForce(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.name='ControllerForce'
        self.node = kw["node"]
        return

    def onKeypressedEvent(self, e):
        inputvalue = self.node.aCable.value
        force = 0
        if e["key"] == Key.plus:
            force = inputvalue.value[0] + 1000.0
        elif e["key"] == Key.minus:
            force = inputvalue.value[0] - 1000.0
            if force < 0:
                force = 0

        inputvalue.value = [force]
        return
