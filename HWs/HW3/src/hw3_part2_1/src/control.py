#!/usr/bin/python3

# Imports
import rospy
import numpy as np
import math, angles
from math import atan2
from itertools import cycle
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


def rect():
    rect = []
    x1 = np.linspace(3,-3,15)
    rect += [[i,2.0] for i in x1]
    y1 = np.linspace(2,-2,10)
    rect += [[-3.0,i] for i in y1]
    rect += [[i,-2.0] for i in x1[::-1]]
    rect += [[3.0,i] for i in y1[::-1]]
    return rect


def spiral():
    spiral = []
    a = 0.17
    growth_factor = math.tan(a)
    for i in range(400):
        t = i / 20 * 3.14
        x = a * math.e ** (growth_factor * t) * math.cos(t)
        y = a * math.e ** (growth_factor * t) * math.sin(t)
        spiral.append([x, y])
    return spiral


def interpolate(p1, p2, n):
    return zip(np.linspace(p1[0], p2[0], n), np.linspace(p1[1], p2[1], n))


def star():
    star = []
    R, r = 6, 3
    star_vertices = []
    for i in range(5):
        star_vertices.append([R * math.cos( math.radians(72 * i)), R * math.sin(math.radians(72 * i))])
        star_vertices.append([r * math.cos(math.radians(36 + 72 * i)), r * math.sin(math.radians(36 + 72 * i))])
    for i in range(len(star_vertices) - 1):
        star += interpolate(star_vertices[i],star_vertices[i + 1], 8)
    star += interpolate(star_vertices[-1],star_vertices[0], 8)
    return star


class PIDWorker():

    def __init__(self, name: str, initial_pid: tuple, initial_error: float, dt: float):
        """ PID worker constructor """
        self.name = name
        self.dt = dt
        self.sum_i = 0
        self.k_p, self.k_i, self.k_d = initial_pid
        self.error = initial_error


    def calculate(self, err:float) -> float:
        self.sum_i += err * self.dt
        # Calculate P
        P = self.k_p * err
        # Calculate I
        I = self.k_i * self.sum_i
        # Calculate D
        D = self.k_d * (err - self.error)
        # Calculate the new error
        self.error = err
        return P + I + D



class PIDController():

    def __init__(self, shape: float = "rect"):
        self.angular_errs = []
        self.shape = shape

        rospy.init_node(name="controller", anonymous=False)
        self.cmd_vel = rospy.Publisher(name="/cmd_vel", data_class=Twist, queue_size=10)

        self.dt = 0.0005
        self.v = 0.3
        rate = 1 / self.dt  # Should be fair enough
        self.target = [0, 0]

        self.mover = None
        self.mover = {
            "star": star(),
            "spiral": spiral(),
            "rect": rect()
        }[self.shape]
        self.infinity_shape = cycle(self.mover)

        rospy.on_shutdown(self.plot_errors) # shutdown callback
        self.r = rospy.Rate(rate)
        self.pose = None 


    def update_pose(self):
        """ Update odometry """
        odom_pos : Odometry = rospy.wait_for_message("/odom", topic_type=Odometry)
        self.pose = odom_pos.pose.pose


    def angle_diff_from_target(self):
        """ Calculate angle diff from target """
        return atan2((self.target[1] - self.pose.position.y),(self.target[0] - self.pose.position.x))


    def distance_diff_from_target(self):
        """ Calculate linear diff from target """
        return math.dist(self.target, [self.pose.position.x, self.pose.position.y])


    def move(self):
        """ Driver method """
        self.update_pose()
        d = self.angle_diff_from_target()
        angular_pid = PIDWorker(name="angular", initial_pid=(0.6, 0.005, 7), initial_error=0, dt=self.dt)
        # linear_pid = PIDWorker(name="linear", initial_pid=(0.15, 0.00, 1), initial_error=0, dt=self.dt)

        # Build Twist message
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            _, _, yaw = euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w])

            err = angles.shortest_angular_distance(yaw, d)  # Find min error
            self.angular_errs.append(err)                   # Save error

            move_cmd.linear.x = self.v
            move_cmd.angular.z = angular_pid.calculate(err)

            self.update_pose()
            if self.distance_diff_from_target() <= 0.3:
                print("Destination Reached, moving to the next one...")
                self.target = next(self.infinity_shape) # Go to next target

            # self.update_pose()                    # Update
            d = self.angle_diff_from_target()       # Calculate new angular diff
            self.r.sleep()


    def plot_errors(self):
        """ Plots robot error """
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.angular_errs))), self.angular_errs)
        plt.axhline(y=0)
        plt.draw()
        plt.legend()
        plt.show()


if __name__ == '__main__':

    shape = str(rospy.get_param("shape", default="spiral"))

    try:
        pidc = PIDController(shape)
        pidc.move()
    except rospy.ROSInterruptException:
        rospy.loginfo("We are done!")
