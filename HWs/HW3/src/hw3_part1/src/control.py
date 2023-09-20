#!/usr/bin/python3

# Imports
import rospy
import math, angles
from math import atan2
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class PIDWorker():

    def __init__(self, name: str, initial_pid: tuple, initial_error: float, dt: float, strategy: str = "PID"):
        """ PID worker constructor """
        self.name = name
        self.k_p, self.k_i, self.k_d = initial_pid
        self.dt = dt
        self.sum_i = 0
        self.error = initial_error
        self.strategy = strategy


    def calculate(self, err: float):
        """ Calculates the PID given the strategy """
        self.sum_i += err * self.dt
        # Calculate P
        P = self.k_p * err if "P" in self.strategy else 0
        # Calculate I
        I = self.k_i * self.sum_i if "I" in self.strategy else 0
        # Calculate D
        D = self.k_d * (err - self.error) if "D" in self.strategy else 0
        rospy.loginfo(f"P:{P}\tI:{I}\tD:{D}")
        # Save error
        self.error = err
        return P + I + D    # Return Total P/I/D



class PIDController():

    def __init__(self, strategy: float = "PID"):
        self.angular_errs = []
        self.strategy = strategy

        rospy.init_node(name="controller", anonymous=False)
        self.cmd_vel = rospy.Publisher(name="/cmd_vel", data_class=Twist, queue_size=10)

        self.dt = 0.0005
        self.v = 0.5
        self.target = [10, 0]
        rate = 1 / self.dt  # Should be fair enough

        rospy.on_shutdown(self.plot_errors) # shutdown callback
        self.r = rospy.Rate(rate)
        self.pose = None 


    def update_pose(self):
        """ Update odometry """
        odom_pos : Odometry = rospy.wait_for_message(topic="/odom", topic_type=Odometry)
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
        angular_pid = PIDWorker(name="angular", initial_pid=(0.6, 0.005, 7), initial_error=0, dt=self.dt, strategy=self.strategy)
        linear_pid = PIDWorker(name="linear", initial_pid=(0.15, 0.00, 1), initial_error=0, dt=self.dt, strategy=self.strategy)

        # Build Twist message
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            _, _, yaw = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
            
            err = angles.shortest_angular_distance(yaw, d)  # Find min error
            self.angular_errs.append(err)                   # Save error

            move_cmd.linear.x = linear_pid.calculate(self.distance_diff_from_target())      # Calculate linear linear PID
            move_cmd.angular.z = angular_pid.calculate(err)                                 # Calculate angular linear PID

            self.update_pose()              # Update
            d = self.angle_diff_from_target()    # Calculate new angular diff
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

    # strategy =  rospy.get_param("strategy")
    strategy = "PID"
    strategy = "PI"
    strategy = "P"

    try:
        pidc = PIDController(strategy)
        pidc.move()
    except rospy.ROSInterruptException:
        rospy.loginfo("We are done!")
