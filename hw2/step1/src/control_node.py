#!/home/mahdi255/.pyenv/shims/python

from math import radians, degrees, atan2, sqrt
import rospy
import tf

from step1.srv import GetDestination
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Controller:
    def __init__(self) -> None:
        rospy.init_node("Control_Node")
        rospy.wait_for_service("Get_Destination")
        self.get_destination = rospy.ServiceProxy("Get_Destination", GetDestination)

        self.cmd_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.linear_speed = rospy.get_param("Control_Node/linear_speed")  # m/s
        self.angular_speed = rospy.get_param("Control_Node/angular_speed")  # rad/s
        self.epsilon = rospy.get_param("Control_Node/epsilon")

        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)

    def get_pose(self):
        msg = rospy.wait_for_message("odom", Odometry)

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        _, _, yaw = tf.transformations.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )

        return position.x, position.y, yaw

    def run(self):
        mean_err = 0
        current_x, current_y, theta = self.get_pose()
        res = self.get_destination(current_x, current_y)

        for i in range(5):
            x = res.next_x - current_x
            y = res.next_y - current_y

            new_theta = atan2(y, x)
            goal_angle = new_theta - theta

            if abs(goal_angle) > radians(180):
                goal_angle += -radians(360) if goal_angle > 0 else radians(360)

            rem = abs(goal_angle)
            prev_angle = theta

            print(f"start iteration {i+1}")
            print(f"current pos: {current_x} {current_y} {degrees(theta)}")
            print(f"destination pos: {res.next_x} {res.next_y} {degrees(new_theta)}")
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
            current_x, current_y, theta = self.get_pose()
            print(f"current pos: {current_x} {current_y} {degrees(theta)}")
            print(f"destination pos: {res.next_x} {res.next_y} {degrees(new_theta)}")
            if abs(theta - new_theta) >= self.angular_speed:
                print("Error in agnle detection")
                current_x, current_y, theta = self.get_pose()
                print(f"error current pos: {current_x} {current_y} {degrees(theta)}")
                continue
            print()

            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_publisher.publish(twist)
            distance_rem = sqrt(pow(x, 2) + pow(y, 2))

            while distance_rem > self.epsilon:
                next_x, next_y, _ = self.get_pose()

                x = next_x - current_x
                y = next_y - current_y

                current_x = next_x
                current_y = next_y

                distance_rem -= sqrt(pow(x, 2) + pow(y, 2))

            self.cmd_publisher.publish(Twist())
            rospy.sleep(5 * self.linear_speed)

            current_x, current_y, theta = self.get_pose()
            self.cmd_publisher.publish(Twist())
            print("finish linear move")
            print(f"current pos: {current_x} {current_y} {degrees(theta)}")
            print(f"destination pos: {res.next_x} {res.next_y} {degrees(new_theta)}")
            distance_err = sqrt(
                pow(current_x - res.next_x, 2) + pow(current_y - res.next_y, 2)
            )
            mean_err += distance_err
            print(f"distance err {distance_err} m")
            print(f"end iteration {i+1}")
            print("=====================================================")
            print()
            res = self.get_destination(current_x, current_y)

        mean_err /= 5
        print(f"mean err {mean_err}")


if __name__ == "__main__":
    c = Controller()
    print("Enter to Start:")
    input()
    c.run()
