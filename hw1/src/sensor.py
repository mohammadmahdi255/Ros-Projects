#!/home/mahdi255/.pyenv/shims/python


import rospy
from hw1.msg import proximity
import random


def distance_sensor():
    topic_distance = rospy.Publisher("distance", proximity, queue_size=10)
    rospy.init_node("distance_sensor")
    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        msg = proximity()
        msg.up = random.randint(10, 200)
        msg.right = random.randint(10, 200)
        msg.down = random.randint(10, 200)
        msg.left = random.randint(10, 200)
        rospy.loginfo(f"\n{msg}")
        topic_distance.publish(msg)
        rate.sleep()
        input()


if __name__ == "__main__":
    distance_sensor()
