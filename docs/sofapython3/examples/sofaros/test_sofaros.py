# coding: utf8
import sofaros
from std_msgs.msg import Float32MultiArray  # wrapper for ROS primitive types, see : https://github.com/ros2/common_interfaces/tree/master/std_msgs


def send(data):
    msg = Float32MultiArray()
    msg.data = list(data.value[0])
    return msg


def recv(data, datafield):
    t = data.tolist()
    datafield.value = [[t[0] + 1.0, t[1], t[2]]]


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin',
                       pluginName=["Sofa.Component.ODESolver.Backward", "Sofa.Component.SceneUtility"])
    rootNode.addObject("EulerImplicitSolver")
    rootNode.addObject("CGLinearSolver", iterations=25, threshold=1e-5, tolerance=1e-5)
    rootNode.addObject('DefaultAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    s = rootNode.addChild("simulated")
    s.addObject("MechanicalObject", name="sender", position=[0.0, 0.0, 0.0])
    s.sender.showObject = True
    s.sender.showObjectScale = 1.0
    s.sender.drawMode = 1

    a = rootNode.addChild("animated")
    a.addObject("MechanicalObject", name="receiver", position=[0.0, 0.0, 0.0])
    a.receiver.showObject = True
    a.receiver.showObjectScale = 1.0
    a.receiver.drawMode = 1
    a.receiver.showColor = [0,1,0,1]

    rosNode = sofaros.init("SofaNode")
    rootNode.addObject(sofaros.RosReceiver(rosNode, "/animation/receiver/position",
                                           a.receiver.findData("position"), Float32MultiArray, recv))

    rootNode.addObject(sofaros.RosSender(rosNode, "/simulation/sender/position",
                                         s.sender.findData("position"), Float32MultiArray, send))

    return rootNode
