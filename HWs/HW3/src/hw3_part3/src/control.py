#!/usr/bin/python3

# Imports
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


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


    def __init__(self):
        self.angular_errs = []

        rospy.init_node(name="controller", anonymous=False)
        self.cmd_vel = rospy.Publisher(name="/cmd_vel", data_class=Twist, queue_size=10)

        # Init PID coeds
        self.dt = 0.006
        self.v = 0.5
        self.D = 1
        rate = 1 / self.dt  # Should be fair enough

        rospy.on_shutdown(self.plot_errors) # shutdown callback
        self.r = rospy.Rate(rate)


    def distance_from_wall(self):
        """ Returns the minimum distance to wall """
        laser_data = rospy.wait_for_message(topic="/scan" , topic_type=LaserScan)
        return min(laser_data.ranges[:180])


    def move(self):
        """ Driver method """
        d = self.distance_from_wall()
        angular_pid = PIDWorker(name="angular", initial_pid=(0.6, 0.005, 7), initial_error=0, dt=self.dt)

        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)

            err = d - self.D
            self.angular_errs.append(err)

            move_cmd.linear.x = self.v
            move_cmd.angular.z = angular_pid.calculate(err)     
            
            d = self.distance_from_wall()
            self.r.sleep()


    def plot_errors(self):
        """ Plots robot error """
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.angular_errs))), self.angular_errs)
        plt.axhline(y=0)
        plt.draw()
        plt.legend()
            


if __name__ == '__main__':

    try:
        pidc = PIDController()
        pidc.move()
    except rospy.ROSInterruptException:
        rospy.loginfo("We are done!")
