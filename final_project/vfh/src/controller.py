#!/home/mahdi255/.pyenv/shims/python
import math

# ROS
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from vfh.srv import angle

from pid import Pid
from vfh_node import sgn


class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller", anonymous=True)

        self.dt = 0.005
        rate = 1 / self.dt
        self.r = rospy.Rate(rate)

        self.angle_pid = Pid('angle', 0.6, 0, 15, self.dt)
        self.distance_pid = Pid('distance', 0.06, 0.01, 20, self.dt)

        # vfh
        rospy.wait_for_service("/get_angle")
        self.vfh = rospy.ServiceProxy("/get_angle", angle)

        # cmd_vel variables
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.max_speed = 0.4
        self.max_turn_rate = 0.6
        self.move_cmd = Twist()

        self.goal = (4.5, 0.0)
        self.goal_list = [(3.0, 4.5), (0.5, 1.5), (1.5, 6.0), (7.8, 3.0), (13.0, 7.0)]

        # odom variables
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.robot_pose_x = 0
        self.robot_pose_y = 0
        self.robot_yaw = 0

    def odom_callback(self, odom_msg):
        self.robot_pose_x = odom_msg.pose.pose.position.x
        self.robot_pose_y = odom_msg.pose.pose.position.y
        orientation = odom_msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def run(self) -> None:
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = 0
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(move_cmd)

            x = self.goal[0] - self.robot_pose_x
            y = self.goal[1] - self.robot_pose_y

            theta_target = math.atan2(y, x) - self.robot_yaw

            if abs(theta_target) > math.pi:
                theta_target += (1 - 2 * sgn(theta_target)) * math.pi
            else:
                theta_target += math.pi

            angle_err = self.vfh(theta_target).radians
            distance_err = math.dist((self.robot_pose_x, self.robot_pose_y), self.goal)
            distance_err = min(distance_err, 2)

            if distance_err <= 0.5:
                if len(self.goal_list) == 0:
                    self.cmd_vel_pub.publish(Twist())
                    return
                self.goal = self.goal_list.pop(0)
                self.distance_pid.sum_i_theta = 0
                distance_err = 0

            if abs(angle_err) < 0.1:
                angle_err = 0

            u_angle = self.angle_pid.calculate(angle_err)
            u_distance = self.distance_pid.calculate(distance_err + 0.5)

            u_angle = sgn(u_angle) * min(abs(u_angle), self.max_turn_rate)

            turn_rate_factor = math.sqrt(self.max_turn_rate - abs(u_angle)) / math.sqrt(self.max_turn_rate)
            u_distance = sgn(u_distance) * min(abs(u_distance), self.max_speed) * turn_rate_factor

            move_cmd.angular.z = u_angle

            if abs(u_angle) > self.max_turn_rate * 0.75:
                self.distance_pid.sum_i_theta *= 0.5

            move_cmd.linear.x = u_distance

            self.r.sleep()


if __name__ == "__main__":
    controller = Controller()
    input("Wait for start")
    controller.run()
