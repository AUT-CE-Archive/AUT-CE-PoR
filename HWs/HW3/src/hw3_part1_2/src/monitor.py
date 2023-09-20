#!/usr/bin/python3

# Imports
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import  PoseStamped


class PathMonitor:

    def __init__(self):

        rospy.init_node("monitor", anonymous=False)
        self.path = Path()
        self.odom_subscriber = rospy.Subscriber(name="/odom", data_class=Odometry, callback=self.odom_callback_handler)
        self.path_subscriber = rospy.Publisher(name="/path", data_class=Path, queue_size=10)


    def odom_callback_handler(self, msg:Odometry):
        """ Odometry callback handler """
        self.path.header = msg.header

        # Construct Pose Stamped message
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)    # Publish pose
        self.path_subscriber.publish(self.path)



if __name__ == "__main__":
    pathMonitor = PathMonitor()
    rospy.spin()    # Smarter
