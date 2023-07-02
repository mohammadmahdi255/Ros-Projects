#!/home/mahdi255/.pyenv/shims/python

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from yolo_step2.srv import WallDetection, WallDetectionResponse
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path

from math import radians

import numpy as np


class Controller:
    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.2
        self.freeze = Twist()

        self.is_left_wall = False
        self.is_right_wall = False

        self.r = rospy.Rate(200)

        # Create a service proxy for your human detection service
        # wait for the service to be available
        rospy.wait_for_service('/detection_wall')
        self.detect_wall = rospy.ServiceProxy('/detection_wall', WallDetection)

        # Create a publisher for your robot "cmd_vel"
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.path = Path()

        self.k_i = 0.0
        self.k_p = 0.6
        self.k_d = 7
        self.D = 0.35
        self.v = 0.2
        self.dt = 0.005

    def distance_from_left_wall(self):
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        rng = laser_data.ranges[:180]
        d = min(rng)
        return d

    def distance_from_front_wall(self):
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        d = laser_data.ranges[0]
        return d

    def update_path(self):
        msg = rospy.wait_for_message("odom", Odometry)

        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

    def run(self) -> None:

        input("start:")

        d = self.distance_from_left_wall()
        sum_i_theta = 0
        prev_theta_error = 0

        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v

        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(move_cmd)
            result: WallDetectionResponse = self.detect_wall()

            d1 = self.distance_from_front_wall()
            self.update_path()

            if d1 < 0.5:
                move_cmd.linear.x = 0
                if d < self.D:
                    self.rotate(radians(95))
                else:
                    self.rotate(radians(95), isClockwise=True)
                continue

            err = d - self.D
            sum_i_theta += err * self.dt

            P = self.k_p * err
            I = self.k_i * sum_i_theta
            D = self.k_d * (err - prev_theta_error)

            move_cmd.angular.z = P + I + D
            prev_theta_error = err
            move_cmd.linear.x = self.v

            d = self.distance_from_left_wall()

            self.r.sleep()

    def rotate(self, angle, isClockwise=True):
        # TODO: code for rotating robot for angle using /cmd_vel topic

        # Set the angular z velocity to rotate the robot clockwise or counterclockwise
        angular_z = 0.5 * (-1 if isClockwise else 1)

        # Define the Twist message with the angular z velocity and zero linear velocity
        twist_msg = Twist()
        twist_msg.angular.z = angular_z

        duration = abs(angle) / abs(angular_z)
        rate = rospy.Rate(200)
        start_time = rospy.Time.now()
        self.cmd_vel_pub.publish(twist_msg)
        while (rospy.Time.now() - start_time).to_sec() < duration:
            rate.sleep()

        # Stop the robot's rotation by sending a Twist message with zero angular z velocity
        self.cmd_vel_pub.publish(self.freeze)


def sgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)

    controller = Controller()
    controller.run()
