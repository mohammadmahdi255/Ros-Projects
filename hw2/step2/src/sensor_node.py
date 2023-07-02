#!/home/mahdi255/.pyenv/shims/python


from math import radians, degrees
import rospy
from sensor_msgs.msg import LaserScan
from step2.msg import obstacle


class Sensor:
    def __init__(self) -> None:
        rospy.init_node("Sensor_Node", anonymous=False)

        self.laser_subscriber = rospy.Subscriber(
            "scan", LaserScan, callback=self.laser_callback
        )

        self.closest_obstacle = rospy.Publisher(
            "closest_obstacle", obstacle, queue_size=10
        )

    def laser_callback(self, msg: LaserScan):
        closest = obstacle()

        index = 0

        for i, r in enumerate(msg.ranges):
            if msg.ranges[index] > r:
                index = i

        closest.distnace = msg.ranges[index]

        if index * msg.angle_increment > radians(180):
            closest.direction = index * msg.angle_increment - radians(360)
        else:
            closest.direction = index * msg.angle_increment

        self.closest_obstacle.publish(closest)

    def run(self):
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":
    Sensor().run()
