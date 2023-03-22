#!/usr/bin/env python

import rospy
import time
from ros_tcp_endpoint.server import TcpServer, SysCommands
from ros_tcp_endpoint.publisher import RosPublisher
from ros_tcp_endpoint.subscriber import RosSubscriber
from ros_tcp_endpoint.service import RosService

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped


def main():
    node_name = rospy.get_param("/TCP_NODE", "TCP_Server")
    rospy.init_node(node_name, anonymous=True)
    tcp_server = TcpServer(node_name)

    tcp_server.start()

    rospy.spin()


if __name__ == "__main__":
    main()

