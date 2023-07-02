#!/home/mahdi255/.pyenv/shims/python

from math import radians, degrees, sqrt
import rospy
import tf
from step2.msg import obstacle
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Controller:
    def __init__(self) -> None:
        rospy.init_node("Control_Node")

        self.cmd_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.linear_speed = rospy.get_param("Control_Node/linear_speed")  # m/s
        self.angular_speed = rospy.get_param("Control_Node/angular_speed")  # rad/s
        self.angle = rospy.get_param("Control_Node/angle")  # rad
        self.epsilon = rospy.get_param("Control_Node/epsilon")

        self.GO, self.ROTATE, self.MUST_GO = 0, 1, 2
        self.state = self.GO
        self.is_rotated = False

        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)

    def get_obstacle(self) -> obstacle:
        return rospy.wait_for_message("closest_obstacle", obstacle)

    def get_pose(self):
        msg = rospy.wait_for_message("odom", Odometry)

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        _, _, yaw = tf.transformations.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )

        return position.x, position.y, yaw

    def run(self):
        while not rospy.is_shutdown():
            if self.state != self.ROTATE:
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)

                if self.state == self.MUST_GO:
                    rem = self.linear_speed
                    x, y, _ = self.get_pose()
                    while rem > self.epsilon:
                        current_x, current_y, _ = self.get_pose()
                        rem -= sqrt(pow(x - current_x, 2) + pow(y - current_y, 2))
                        x, y = current_x, current_y
                        msg = self.get_obstacle()
                        if msg.distnace < 0.4:
                            break

                    self.state = self.GO
                    continue

                msg = self.get_obstacle()
                if msg.distnace < 2:
                    _, _, theta = self.get_pose()
                    self.angle = (
                        msg.direction + theta - radians(180)
                        if msg.direction + theta > 0
                        else msg.direction + theta + radians(180)
                    )

                    self.state = self.ROTATE

                continue

            self.cmd_publisher.publish(Twist())
            rospy.sleep(1)

            current_x, current_y, theta = self.get_pose()

            new_theta = self.angle
            goal_angle = new_theta - theta

            if abs(goal_angle) > radians(180):
                goal_angle += -radians(360) if goal_angle > 0 else radians(360)

            rem = abs(goal_angle)
            prev_angle = theta

            print(f"current pos: {current_x} {current_y} {degrees(theta)}")
            print(f"destination angle: {degrees(new_theta)}")
            print(f"rotate:{degrees(rem)}")
            print()

            twist = Twist()
            twist.angular.z = abs(self.angular_speed) * (abs(goal_angle) / goal_angle)
            self.cmd_publisher.publish(twist)

            while rem > self.epsilon:
                _, _, theta = self.get_pose()
                delta = abs(prev_angle - theta)
                if abs(delta) > radians(180):
                    delta = radians(360) - delta
                rem -= delta
                prev_angle = theta

            self.cmd_publisher.publish(Twist())
            rospy.sleep(5 * self.angular_speed)

            print("finish rotation move")
            _, _, theta = self.get_pose()
            print(f"current angle: {degrees(theta)}")
            print(f"destination angle: {degrees(new_theta)}")
            print(f"subtract angle: {abs(theta - new_theta)}")
            print(f"rotate:{degrees(rem)}")
            print("=========================================")
            if abs(theta - new_theta) >= self.angular_speed:
                _, _, theta = self.get_pose()
                print("Error in agnle detection")
                print(f"error current pos: {degrees(theta)}")
                print("=========================================")
                continue

            self.state = self.MUST_GO


if __name__ == "__main__":
    c = Controller()
    print("Enter to start")
    input()
    c.run()
