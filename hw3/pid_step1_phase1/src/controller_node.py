#!/home/mahdi255/.pyenv/shims/python

from math import sqrt, atan2, radians, degrees
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import tf
from pid import pid


class Controller():

    def __init__(self):

        rospy.init_node('controller', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.dt = 0.005

        self.threshold = 0.01

        self.angle_pid = pid('angle', 1, 0.01, 10, self.dt)
        self.distance_pid = pid('distance', 0.3, 0.01, 3, self.dt)

        rate = 1/self.dt

        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []

    def get_pose(self):
        msg = rospy.wait_for_message("odom", Odometry)

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

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

            if -radians(90) < a_err < radians(90):
                d_err = sqrt(pow(pose.x - goal_x, 2) + pow(pose.y - goal_y, 2))
            else:
                d_err = -sqrt(pow(pose.x - goal_x, 2) +
                              pow(pose.y - goal_y, 2))

            if abs(d_err) < self.threshold:
                self.cmd_vel.publish(Twist())
                rospy.signal_shutdown(reason="reached goal")

            u_angle = self.angle_pid.calculate(a_err)
            u_distance = self.distance_pid.calculate(d_err)

            rospy.loginfo(
                f"d_err: {d_err},  theta: {degrees(theta)} yaw: {degrees(yaw)}, u_distance: {u_distance} u_angle: {u_angle}")

            u_angle = u_angle / abs(u_angle) * min(abs(u_angle), 0.8)
            move_cmd.angular.z = u_angle

            u_distance = u_distance / \
                abs(u_distance) * min(abs(u_distance), 0.5)
            move_cmd.linear.x = u_distance

            pose, yaw = self.get_pose()

            self.r.sleep()

    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        plt.subplot(2, 1, 1)
        plt.plot(list(range(len(self.angle_pid.errs))),
                 self.angle_pid.errs, label='angle_errs')
        plt.axhline(y=0, color='red')
        plt.legend(loc="upper left", frameon=False)

        plt.subplot(2, 1, 2)
        plt.plot(list(range(len(self.distance_pid.errs))),
                 self.distance_pid.errs, label='distance_errs')
        plt.axhline(y=0, color='red')
        plt.legend(loc="upper left", frameon=False)
        plt.show()

        plt.close()


if __name__ == '__main__':
    try:
        pidc = Controller()
        input()
        pidc.goal_arrive(10, 0)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
