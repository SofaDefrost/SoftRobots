#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

pub = None


def callback(data):
    global pub
    pub.publish(data)
    print(data)


if __name__ == '__main__':
    try:
        rospy.init_node('listener', anonymous=True)
        pub = rospy.Publisher("/animation/receiver/position", numpy_msg(Floats), queue_size=10)
        rospy.Subscriber("/simulation/sender/position", numpy_msg(Floats), callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
