import Sofa.Core
import Sofa.constants.Key as Key

class ControllerVolume(Sofa.Core.Controller):

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        return

    def onKeypressedEvent(self, e):
        inputvalue = self.node.cavity.surfaceConstraint.value

        volume = 0
        if e["key"] == Key.plus:
            volume = inputvalue.value[0] + 5.0
        elif e["key"] == Key.minus:
            volume = inputvalue.value[0] - 5.0
            if volume < 0:
                volume = 0

        inputvalue.value = [volume]
        return
