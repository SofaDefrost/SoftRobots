# coding: utf8
import Sofa
import rospy


class RosSender(Sofa.PythonScriptController):
    def __init__(self, node, rosname, datafield, msgtype, sendingcb):
        # to reduce the latency in TCP, disable Nagle's algo with tcp_nodelay=False
        #  (https://en.wikipedia.org/wiki/Nagle%27s_algorithm)
        self.pub = rospy.Publisher(rosname, msgtype, queue_size=10, tcp_nodelay=False)
        self.sendingcb = sendingcb
        self.datafield = datafield

    def onBeginAnimationStep(self, dt):
        self.pub.publish(self.sendingcb(self.datafield))


class RosReceiver(Sofa.PythonScriptController):
    def __init__(self, node, rosname, datafield, msgtype, recvcb):
        rospy.Subscriber(rosname, msgtype, self.callback)
        self.data = None
        self.datafield = datafield
        self.recvcb = recvcb

    def callback(self, data):
        self.data = data.data

    def onBeginAnimationStep(self, dt):
        if self.data is not None:
            self.recvcb(self.data, self.datafield)
            self.data = None


def init(nodename="Sofa"):
    rospy.init_node(nodename, anonymous=True)
