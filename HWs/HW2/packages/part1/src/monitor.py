#!/usr/bin/python3

import rospy
import tf

from part1.srv import GetNextDestination

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
import math


class PathMonitor:

    def __init__(self):
        rospy.init_node("monitor", anonymous=False)
        self.path = Path()
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, callback=self.odom_callback)
        self.path_subscriber = rospy.Publisher("/path", Path, queue_size=10)

    def odom_callback(self, msg:Odometry):
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_subscriber.publish(self.path)


if __name__ == "__main__":
    pathMonitor = PathMonitor()
    rospy.spin()
