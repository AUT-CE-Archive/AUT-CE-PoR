#!/usr/bin/python3

# Imports
import rospy, random, math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from hw3_part1_2.srv import GetNextDestination, GetNextDestinationResponse


def getr_next_destination(x: int, y: int):
    while True:
        next = (random.uniform(-20, 20),random.uniform(-20, 20))
        if math.dist((x, y), next) >= 10:
            return next


def get_next_destination_callback_handler(req: GetNextDestination):
    # Current x, y
    _x, _y = req.current_x, req.current_y
    # next x,y 
    x, y = getr_next_destination(_x,_y)

    # Build and retunr response
    res = GetNextDestinationResponse()
    res.next_x = x
    res.next_y = y
    return res


def listener():
    rospy.init_node(name="Mission", anonymous=False)
    _ = rospy.Service(name="/get_next_destination", service_class=GetNextDestination, handler=get_next_destination_callback_handler)
    rospy.spin()    # Work smarker, not harder!



if __name__ == '__main__':
    listener()
