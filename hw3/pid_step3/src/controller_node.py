#!/home/mahdi255/.pyenv/shims/python

from math import *
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
import matplotlib.pyplot as plt
import tf
from pid import pid


class Controller():

    def __init__(self):

        rospy.init_node('controller', anonymous=False)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.path = Path()

        self.dt = 0.005
        self.threshold = 0.01
        self.v = 0.4
        self.D = 2
        self.is_right = False

        self.angle_pid = pid('angle', 0.8, 0.001, 16, self.dt)

        rate = 1/self.dt

        self.X = []
        self.Y = []

        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []

    def get_pose(self):
        msg = rospy.wait_for_message("odom", Odometry)

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

        _, _, yaw = tf.transformations.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )

        return position, yaw

    def distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        left = laser_data.ranges[:180]
        rigth = laser_data.ranges[180:]

        if self.is_right:
            d = min(rigth)
        else:
            d = min(left)

        if isinf(d):
            self.is_right = not self.is_right

        return d

    def goal_arrive(self):

        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = 0

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)

            d = self.distance_from_wall()
            a_err = d - self.D

            if not isinf(d):
                u_angle = self.angle_pid.calculate(a_err)
            else:
                u_angle = 0

            # rospy.loginfo(
            #     f"d_err: {d_err},  theta: {degrees(theta)} yaw: {degrees(yaw)}, u_distance: {u_distance} u_angle: {u_angle}")

            # rospy.loginfo(u_angle)
            u_angle = u_angle / abs(u_angle) * min(abs(u_angle), 0.8)
            move_cmd.angular.z = u_angle
            move_cmd.linear.x = self.v * (1 - abs(u_angle))
            # rospy.loginfo(move_cmd.linear.x)

            pose, _ = self.get_pose()
            self.X.append(pose.x)
            self.Y.append(pose.y)

            self.r.sleep()

        self.angle_pid.reset()

    def plot(self):
        plt.plot(self.X, self.Y)
        plt.show()

        plt.close()


def find_closest_point(points):

    if points is None or points.shape[0] == 0:
        return None

    pose, _ = pidc.get_pose()
    point = points[0]

    for p in points:
        if pow(pose.x - point[0], 2) + pow(pose.y - point[1], 2) > pow(pose.x - p[0], 2) + pow(pose.y - p[1], 2):
            point = p

    return point


def distance(p1, p2):
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2))


if __name__ == '__main__':
    try:
        pidc = Controller()
        input()
        pidc.goal_arrive()
        pidc.plot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
