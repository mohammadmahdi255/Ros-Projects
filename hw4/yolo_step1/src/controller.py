#!/home/mahdi255/.pyenv/shims/python

# ROS
import rospy
from geometry_msgs.msg import Twist
from yolo_step1.srv import PersonDetection

from pid import pid


class Controller:
    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()

        self.dt = 0.005

        self.angle_pid = pid('angle', 0.01, 0, 5, self.dt)
        self.distance_pid = pid('distance', 0.1, 0, 5, self.dt)

        rate = 1/self.dt
        self.r = rospy.Rate(rate)

        # Create a service proxy for your human detection service
        # wait for the service to be available
        rospy.wait_for_service('/detection')
        self.detect_human = rospy.ServiceProxy('/detection', PersonDetection)

        # Create a publisher for your robot "cmd_vel"
        self.cmd_vel_pub = rospy.Publisher(
            '/follower/cmd_vel', Twist, queue_size=10)

    def run(self) -> None:
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = 0
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(move_cmd)
            result = self.detect_human('person')
            center_x = (result.x1 + result.x2) / 2
            center_y = abs(result.y1 - result.y2)

            angle_err = (160 - center_x)
            distance_err = (100 - center_y)

            if abs(distance_err) < 5:
                distance_err = 0

            if abs(angle_err) < 5:
                angle_err = 0

            u_angle = self.angle_pid.calculate(angle_err)
            u_distance = self.distance_pid.calculate(distance_err)

            u_angle = sgn(u_angle) * min(abs(u_angle), 0.3)
            move_cmd.angular.z = u_angle

            u_distance = sgn(u_distance) * min(abs(u_distance), 0.2)
            move_cmd.linear.x = u_distance

            self.r.sleep()


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
