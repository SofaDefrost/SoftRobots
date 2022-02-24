import Sofa.Core
import Sofa.constants.Key as Key


class ControllerPressure(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        return

    def onKeypressedEvent(self, e):
        inputvalue = self.node.cavity.surfaceConstraint.value

        pressure = 0
        if e["key"] == Key.plus:
            pressure = inputvalue.value[0] + 1.0
        elif e["key"] == Key.minus:
            pressure = inputvalue.value[0] - 1.0
            if pressure < 0:
                pressure = 0

        inputvalue.value = [pressure]
        return
