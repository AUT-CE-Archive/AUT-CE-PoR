#!/usr/bin/python3

# Imports
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class PathMonitor:

    def __init__(self):
        rospy.init_node("monitor", anonymous=False)
        self.path = Path()
        self.path_subscriber = rospy.Publisher(name="/path", data_class=Path, queue_size=10)
        self.odom_subscriber = rospy.Subscriber(name="/odom", data_class=Odometry, callback=self.odom_callback)


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
