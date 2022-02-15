#!/usr/bin/env python
import rclpy
from std_msgs.msg import Float32MultiArray
import sys

pub = None


def callback(data):
    global pub
    pub.publish(data)
    print(data)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('listener')
    node.get_logger().info('Created node')
    pub = node.create_publisher(Float32MultiArray, "/animation/receiver/position", 10)
    sub = node.create_subscription(Float32MultiArray, "/simulation/sender/position", callback, 10)
    rclpy.spin(node)
