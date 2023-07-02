#!/home/mahdi255/.pyenv/shims/python

from math import *
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
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

        self.angle_pid = pid('angle', 1, 0.001, 6, self.dt)
        self.distance_pid = pid('distance', 0.8, 0.001, 5, self.dt)

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

    def goal_arrive(self, path):

        pose, yaw = self.get_pose()

        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = 0

        point = find_closest_point(path)
        index = np.argwhere(np.all(path == point, axis=1))[0][0]
        counter = 1

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)

            goal_x, goal_y = point[0], point[1]

            theta = atan2(goal_y - pose.y, goal_x - pose.x)

            a_err = theta - yaw
            if a_err > radians(180):
                a_err -= radians(360)
            elif a_err < -radians(180):
                a_err += radians(360)

            if -radians(90) < a_err < radians(90):
                d_err = sqrt(pow(pose.x - goal_x, 2) + pow(pose.y - goal_y, 2))
            else:
                d_err = -sqrt(pow(pose.x - goal_x, 2) +
                              pow(pose.y - goal_y, 2))

            if d_err < self.threshold:
                move_cmd = Twist()
                index = (index + 1) % len(path)
                if counter == len(path) - 1:
                    self.cmd_vel.publish(move_cmd)
                    rospy.loginfo(f"index: {index}")
                    break
                point = path[index]
                counter += 1
                continue

            u_angle = self.angle_pid.calculate(a_err)
            u_distance = self.distance_pid.calculate(d_err)

            # rospy.loginfo(
            #     f"d_err: {d_err},  theta: {degrees(theta)} yaw: {degrees(yaw)}, u_distance: {u_distance} u_angle: {u_angle}")

            u_angle = u_angle / abs(u_angle) * min(abs(u_angle), 0.8)
            move_cmd.angular.z = u_angle

            u_distance = u_distance / \
                abs(u_distance) * min(abs(u_distance), 0.5)

            move_cmd.linear.x = u_distance

            pose, yaw = self.get_pose()
            self.X.append(pose.x)
            self.Y.append(pose.y)

            self.r.sleep()

        pose, yaw = self.get_pose()
        d_err = sqrt(pow(pose.x - goal_x, 2) + pow(pose.y - goal_y, 2))
        rospy.loginfo(f"distance err: {d_err}")

        self.angle_pid.reset()
        self.distance_pid.reset()

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

        a = 0.17
        k = tan(a)
        X, Y = [], []

        for i in range(3000):
            t = i / 400.0 * pi
            dx = a * exp(k * t) * cos(t)
            dy = a * exp(k * t) * sin(t)
            X.append(dx)
            Y.append(dy)

        XY = np.column_stack((X, Y))

        for i in range(1, len(XY)):
            print(distance(XY[i-1], XY[i]))

        pidc.goal_arrive(XY)

        pidc.plot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
