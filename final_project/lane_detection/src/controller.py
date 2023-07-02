#!/home/mahdi255/.pyenv/shims/python
import math

# ROS
import rospy
from geometry_msgs.msg import Twist
from pid import Pid
from std_msgs.msg import Float64, UInt8
from vfh.srv import angle
from vfh_node import sgn


class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller", anonymous=True)
        rospy.Subscriber('/control/lane', Float64, self.cbFollowLane, queue_size=1)
        rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size=1)
        rospy.Subscriber('/detect/traffic_sign', UInt8, self.get_traffic_sign, queue_size=1)

        self.traffic_sign = 0
        self.stop_timer = 0
        self.state = 0

        self.lastError = 0
        self.MAX_VEL = 0.1

        self.dt = 0.005
        rate = 1 / self.dt
        self.r = rospy.Rate(rate)

        self.angle_pid = Pid('angle', 0.6, 0, 15, self.dt)

        # vfh
        rospy.wait_for_service("/get_angle")
        self.vfh = rospy.ServiceProxy("/get_angle", angle)

        # cmd_vel variables
        self.cmd_vel_pub = rospy.Publisher('/control/cmd_vel', Twist, queue_size=1)
        self.max_speed = 0.4
        self.max_turn_rate = 0.6
        self.move_cmd = Twist()

        rospy.on_shutdown(self.fnShutDown)

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def get_traffic_sign(self, msg: UInt8):
        self.traffic_sign = msg.data
        if msg.data == 5:
            rospy.loginfo(f"sign intersection seem")
        elif msg.data == 10:
            rospy.loginfo(f"sign parking seem")
        elif msg.data == 20:
            rospy.loginfo(f"sign construction seem")

    def cbFollowLane(self, desired_center):
        center = desired_center.data

        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error

        angle_err = self.vfh(math.pi).radians
        self.angle_pid.calculate(angle_err)

        if self.traffic_sign == 10 and self.state == 0:
            self.stop_timer = rospy.get_time()
            self.cmd_vel_pub.publish(Twist())
            self.state = 1
            self.traffic_sign = 0
            return
        elif self.state == 1:
            if rospy.get_time() - self.stop_timer >= 5:
                self.state = 2
                self.stop_timer = rospy.get_time()
            return
        elif self.state == 2 and rospy.get_time() - self.stop_timer >= 30:
            self.state = 0

        twist = Twist()
        if self.traffic_sign == 20:
            twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2).real, 0.05) / 2
            self.traffic_sign = 0
        else:
            twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2).real, 0.05)
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = - sgn(angular_z) * min(abs(angular_z), 2.0)
        self.cmd_vel_pub.publish(twist)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    @staticmethod
    def run():
        rospy.spin()


if __name__ == "__main__":
    controller = Controller()
    input("Wait for start")
    controller.run()
