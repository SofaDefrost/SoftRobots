import Sofa
from stlib.scene import MainHeader
from stlib.animation import animate

class FakeServoMotor(Sofa.PythonScriptController):
    def __init__(self, node, name="FakeServoMotor"):
        self.name = name
        self.addNewData("angle", "Properties", "The angle of the servo motor", "d", 0.0)

class SerialPortController(Sofa.PythonScriptController):
    def __init__(self, node, input, serialport):
        self.serialport = serialport

        for i in self.input:
           self.addNewData("angle", "Properties", "The angle of the servo motor", "d", 0.0)


    #def onAnimationEnd(self, dt):
    #    print("On Animation. ")
    #self.serialport.value = [self.input[0], self.input[1], self.input[2]]

def SerialPort(rootNode, serialport="/dev/ttyUSB0"):
    rootNode.createObject("SerialPortBridgeGeneric", port=serialport, baudRate=115200, size=3, listening=True)
    

#Units: cm and kg
def createScene(rootNode):
    MainHeader(rootNode, plugins=["SoftRobots"])

    s1 = FakeServoMotor(rootNode, "s1")
    s2 = FakeServoMotor(rootNode, "s2")
    s3 = FakeServoMotor(rootNode, "s3")

    sp = SerialPort(rootNode)
    spc = SerialPortController(rootNode, [s1,s2,s3], sp)
