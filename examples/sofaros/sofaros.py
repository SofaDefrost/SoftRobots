# coding: utf8
import Sofa.Core
import rclpy


class RosSender(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # to reduce the latency in TCP, we can disable Nagle's algo with tcp_nodelay=False in ROS1
        # (https://en.wikipedia.org/wiki/Nagle%27s_algorithm)
        # Todo: find the equivalent in ROS2
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "RosSender"

        # Args
        self.node = args[0]
        rosname = args[1]
        self.datafield = args[2]
        msgtype = args[3]
        self.sendingcb = args[4]

        # Create or connect to the topic rosname as a publisher
        self.pub = self.node.create_publisher(msgtype, rosname, 10)

    def onAnimateEndEvent(self, event):
        data = self.sendingcb(self.datafield)
        self.pub.publish(data)


class RosReceiver(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "RosReceiver"

        self.node = args[0]
        rosname = args[1]
        self.datafield = args[2]
        msgtype = args[3]
        self.recvcb = args[4]

        # Create or connect to the topic rosname as a subscription
        self.sub = self.node.create_subscription(msgtype, rosname, self.callback, 10)

        self.data = None

    def callback(self, data):
        self.data = data.data

    def onAnimateBeginEvent(self, event):
        rclpy.spin_once(self.node, timeout_sec=0.001)  # Enables callbacks
        if self.data is not None:
            self.recvcb(self.data, self.datafield)
            self.data = None


def init(nodeName="Sofa"):
    rclpy.init()
    node = rclpy.create_node(nodeName)
    node.get_logger().info('Created node')
    return node
