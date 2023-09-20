#!/usr/bin/python3

import math

import rospy
import tf
import numpy as np
from part2.msg import GetDistance
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def get_goal_heading():

    data = rospy.wait_for_message("/scan", LaserScan)
    ranges = data.ranges[0:90] + data.ranges[270:360]
    minimum = min(ranges)
    min_index = ranges.index(minimum)
    if min_index <= 90:
        return -(180 - min_index) * math.pi / 180, minimum
    else:
        y = 180 - min_index
        return (180 - y) * math.pi / 180, minimum


def talker():
    rospy.init_node("sensor", anonymous=False)
    distance_publisher = rospy.Publisher(
        '/ClosestObstacle', GetDistance, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = GetDistance()
        msg.direction, msg.distance = get_goal_heading()
        distance_publisher.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    talker()
