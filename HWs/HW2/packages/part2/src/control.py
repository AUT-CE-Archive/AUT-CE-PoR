#!/usr/bin/python3
import rospy, tf, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from part2.msg import GetDistance


class Contoroller:
    def __init__(self):
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        self.linear_speed = 0.2
        self.angular_speed = 0.2
        self.c_x, self.c_y = 0, 0
        self.avg_dis = 0

    def get_heading(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        ))
        return yaw
    
    def get_status(self):
        msg = rospy.wait_for_message("/ClosestObstacle", GetDistance)
        return msg.direction, msg.distance


    
    def rotate(self, status):
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)
        remaining_angle, clockwise = status
        prev_angle = self.get_heading()
        twist = Twist()
        twist.angular.z = self.angular_speed * clockwise
        self.cmd_publisher.publish(twist)
        while remaining_angle >= 0.002:
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            remaining_angle -= delta
            prev_angle = current_angle
        self.cmd_publisher.publish(Twist())
    
    def go(self):
        while True:
            if self.get_status()[1] < 2:
                break
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_publisher.publish(twist)
            
        twist = Twist()
        self.cmd_publisher.publish(twist)


    def run(self):
        while not rospy.is_shutdown():
            print("GO...")
            self.go()
            deg, _ = self.get_status()
            print("ROTATE...")
            self.rotate(status=(abs(deg), deg / abs(deg)))


if __name__ == "__main__":
    rospy.init_node('control', anonymous=False)
    c = Contoroller()
    c.run()