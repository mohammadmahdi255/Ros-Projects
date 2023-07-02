#!/home/mahdi255/.pyenv/shims/python

from math import sqrt, atan2, radians, degrees
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

        self.angle_pid = pid('angle', 0.8, 0.01, 16, self.dt)
        self.distance_pid = pid('distance', 0.06, 0.01, 10, self.dt)

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

    def goal_arrive(self, goal_x, goal_y):

        pose, yaw = self.get_pose()

        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = 0

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)

            theta = atan2(goal_y - pose.y, goal_x - pose.x)

            a_err = theta - yaw
            if a_err > radians(180):
                a_err -= radians(360)
            elif a_err < -radians(180):
                a_err += radians(360)

            if abs(a_err) < self.threshold:
                move_cmd = Twist()
                rospy.loginfo(f"a_err: {degrees(a_err)}")
                self.cmd_vel.publish(move_cmd)
                break

            u_angle = self.angle_pid.calculate(a_err)
            move_cmd.angular.z = u_angle

            pose, yaw = self.get_pose()

            self.r.sleep()

        rospy.sleep(1)
        self.angle_pid.reset()

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)

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

            if abs(d_err) < self.threshold:
                self.cmd_vel.publish(Twist())
                break

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

        rospy.sleep(1)
        pose, yaw = self.get_pose()
        d_err = sqrt(pow(pose.x - goal_x, 2) + pow(pose.y - goal_y, 2))
        rospy.loginfo(f"distance err: {d_err}")

        self.angle_pid.reset()
        self.distance_pid.reset()

    def plot(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
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


if __name__ == '__main__':
    try:
        pidc = Controller()

        X1 = np.linspace(0, 3, 100)
        Y1 = - (7/3) * X1 + 12

        X2 = np.linspace(3, 10, 100)
        Y2 = np.array([5]*100)

        X3 = np.linspace(10, 4, 100)
        Y3 = (5/6) * X3 - (10/3)

        X4 = np.linspace(4, 7, 100)
        Y4 = -(3) * X4 + 12

        X5 = np.linspace(7, 0, 100)
        Y5 = -(4/7) * X5 - 5

        X6 = np.linspace(0, -7, 100)
        Y6 = (4/7) * X6 - 5

        X7 = np.linspace(-7, -4, 100)
        Y7 = 3 * X7 + 12

        X8 = np.linspace(-4, -10, 100)
        Y8 = -(5/6) * X8 - (10/3)

        X9 = np.linspace(-10, -3, 100)
        Y9 = np.array([5]*100)

        X10 = np.linspace(-3, 0, 100)
        Y10 = (7/3) * X10 + 12

        X = np.concatenate([X1, X2, X3, X4, X5, X6, X7, X8, X9, X10])
        Y = np.concatenate([Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9, Y10])

        XY = np.column_stack((X, Y))

        first = find_closest_point(XY)
        pidc.goal_arrive(first[0], first[1])

        points = np.array([(0, 12), (3, 5), (10, 5),
                           (4, 0), (7, -9), (0, -5),
                           (-7, -9), (-4, 0), (-10, 5),
                           (-3, 5)])
        point = find_closest_point(points)
        index = np.argwhere(np.all(points == point, axis=1))[0][0]

        for _ in range(len(points)):
            index = (index + 1) % points.shape[0]
            point = points[index]
            pidc.goal_arrive(point[0], point[1])

        pidc.plot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
