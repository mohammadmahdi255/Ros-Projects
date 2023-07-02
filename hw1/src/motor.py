#!/home/mahdi255/.pyenv/shims/python

import sys
import rospy
from package.msg import action


def motor_action(data):
    rospy.loginfo(data)


def distance_sensor(node_name: str):
    rospy.Subscriber(node_name, action, motor_action)
    rospy.init_node(node_name)
    rospy.spin()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("wrong number of argument")
    else:
        distance_sensor(sys.argv[1])
