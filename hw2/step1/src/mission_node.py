#!/home/mahdi255/.pyenv/shims/python

import random
import rospy
from step1.srv import GetDestination, GetDestinationResponse


def response(req: GetDestination):
    current_x = req.current_x
    current_y = req.current_y
    next_x, next_y = 0, 0

    while True:
        next_x, next_y = random.uniform(-10, 10), random.uniform(-10, 10)
        if pow(next_x - current_x, 2) + pow(next_y - current_y, 2) >= 25:
            break

    res = GetDestinationResponse()
    res.next_x = next_x
    res.next_y = next_y

    return res


def listener():
    rospy.init_node("Mission_Node")
    rospy.Service("Get_Destination", GetDestination, response)
    rospy.spin()


if __name__ == "__main__":
    listener()
