# coding: utf8
import Sofa.Core
import rclpy


class RosSender(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # to reduce the latency in TCP, disable Nagle's algo with tcp_nodelay=False
        #  (https://en.wikipedia.org/wiki/Nagle%27s_algorithm)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "RosSender"
        node = args[0]
        rosname = args[1]
        self.datafield = args[2]
        msgtype = args[3]
        self.sendingcb = args[4]
        self.pub = node.create_publisher(msgtype, rosname, 10)

    def onAnimateBeginEvent(self, event):
        data = self.sendingcb(self.datafield)
        self.pub.publish(data)


class RosReceiver(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "RosReceiver"
        node = args[0]
        rosname = args[1]
        self.datafield = args[2]
        msgtype = args[3]
        self.recvcb = args[4]
        self.sub = node.create_subscription(msgtype, rosname, self.callback, 10)
        self.data = None

    def callback(self, data):
        self.data = data.data

    def onAnimateBeginEvent(self, event):
        if self.data is not None:
            self.recvcb(self.data, self.datafield)
            self.data = None


def init(nodeName="Sofa"):
    rclpy.init()
    node = rclpy.create_node(nodeName)
    return node
