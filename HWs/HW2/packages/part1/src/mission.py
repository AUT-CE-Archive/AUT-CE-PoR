#!/usr/bin/python3

import rosp
import random

from part1.srv import GetNextDestination, GetNextDestinationResponse


def find_next_destination(c_x, c_y):
    while True:
        x, y = random.uniform(-10, 10), random.uniform(-10, 10)
        if (c_x - x) ** 2 + (c_y - y) ** 2 > 25:
            break
    return x, y

def response(req):
    c_x, c_y = req.current_x, req.current_y
    x, y = find_next_destination(c_x, c_y)
    res = GetNextDestinationResponse()
    res.next_x = x
    res.next_y = y
    return res


def listener():
    rospy.init_node("Mission", anonymous=False)
    s = rospy.Service("/get_next_destination", GetNextDestination, response)
    rospy.spin()



if __name__ == '__main__':
    listener()
