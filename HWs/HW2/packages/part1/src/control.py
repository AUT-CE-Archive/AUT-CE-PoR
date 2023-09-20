#!/usr/bin/python3
import rospy
import math
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from part1.srv import GetNextDestination


class Contoroller:
    def __init__(self):
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.linear_speed = rospy.get_param("/control/linear_speed")  # m/s
        self.angular_speed = 0.2
        self.c_x, self.c_y = 0, 0
        self.avg_dis = 0

    def get_heading(self):
        msg = rospy.wait_for_message("/odom", Odometry)
        orientation = msg.pose.pose.orientation

        # convert quaternion to odom
        _, _, yaw = tf.transformations.euler_from_quaternion((
            orientation.x, orientation.y, orientation.z, orientation.w
        ))
        return yaw

    def get_next_destination(self):
        rospy.wait_for_service("/get_next_destination")
        try:
            new_dest = rospy.ServiceProxy(
                "/get_next_destination", GetNextDestination)
            resp = new_dest(float(self.c_x), float(self.c_y))
            return resp.next_x, resp.next_y
        except rospy.ServiceException as e:
            pass

    def calculate_degree_of_rotation(self, n_x, n_y):
        yaw = self.get_heading()
        dx, dy = n_x - self.c_x, n_y - self.c_y
        angle = math.atan2(dy, dx)
        return abs(yaw - angle), 1 if yaw < angle else -1

    def rotate(self, n_x, n_y):
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)
        remaining_angle, clockwise = self.calculate_degree_of_rotation(
            n_x, n_y)
        prev_angle = self.get_heading()
        twist = Twist()
        twist.angular.z = self.angular_speed * clockwise
        self.cmd_publisher.publish(twist)
        # rotation loop
        while remaining_angle >= 0.002:
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            # if (current_angle<0 and prev_angle>0):
            #     delta = abs(abs(math.pi -prev_angle) + abs(current_angle + math.pi))
            remaining_angle -= delta
            prev_angle = current_angle
        self.cmd_publisher.publish(Twist())

    def go(self, remaining):
        msg = rospy.wait_for_message("/odom", Odometry)
        prev_x, prev_y = msg.pose.pose.position.x, msg.pose.pose.position.y

        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_publisher.publish(twist)
        rospy.sleep(1)
        msg = rospy.wait_for_message("/odom", Odometry)

        while remaining >= 0.01:

            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_publisher.publish(twist)

            msg = rospy.wait_for_message("/odom", Odometry)
            self.c_x, self.c_y = msg.pose.pose.position.x, msg.pose.pose.position.y

            delta = math.dist([self.c_x, self.c_y], [prev_x, prev_y])
            remaining -= delta
            prev_x, prev_y = msg.pose.pose.position.x, msg.pose.pose.position.y

        twist = Twist()
        self.cmd_publisher.publish(twist)

    def verbose(self, n_x, n_y, index):
        print("Destination:", n_x, n_y)
        msg = rospy.wait_for_message("/odom", Odometry)
        print("Reached:", msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.avg_dis += math.dist([n_x, n_y], [self.c_x, self.c_y])
        print(f"Distance = {math.dist([n_x, n_y], [self.c_x, self.c_y])}")
        print("----------------------------")
        if index == 4:
            print(f"Average disance: = {self.avg_dis / 5}")

    def run(self):
        index = 0
        while not rospy.is_shutdown():
            # get location
            n_x, n_y = self.get_next_destination()
            # state is rotation
            self.rotate(n_x, n_y)
            # state is go
            self.go(math.dist([n_x, n_y], [self.c_x, self.c_y]))
            self.verbose(n_x, n_y, index)
            index += 1


if __name__ == "__main__":
    rospy.init_node('control', anonymous=False)
    c = Contoroller()
    c.run()
