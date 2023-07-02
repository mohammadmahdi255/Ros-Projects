#!/home/mahdi255/.pyenv/shims/python

import numpy as np
import rospy
from hw1.msg import action, proximity


class Controller:

    def __init__(self) -> None:
        self.state = "up"
        self.vectors = {
            "up": np.asarray([0, 1, 0]),
            "right": np.asarray([1, 0, 0]),
            "down": np.asarray([0, -1, 0]),
            "left": np.asarray([-1, 0, 0]),
        }

        rospy.Subscriber("distance", proximity, self.action_decision)
        self.topic_motor1 = rospy.Publisher("motor1", action, queue_size=10)
        self.topic_motor2 = rospy.Publisher("motor2", action, queue_size=10)
        rospy.init_node("controller")
        rospy.spin()

    def action_decision(self, data: proximity) -> None:
        min_distance = min(data.up, data.right, data.down, data.left)

        if min_distance == data.up:
            next_state = "down"
        elif min_distance == data.down:
            next_state = "up"
        elif min_distance == data.left:
            next_state = "right"
        else:
            next_state = "left"

        rospy.loginfo(f"{self.state} -> {next_state}\n{data}")

        start_vector = self.vectors[self.state]
        end_vector = self.vectors[next_state]
        self.state = next_state

        outer_product = np.cross(start_vector, end_vector)

        msg = action()
        msg.isClockwise = outer_product[2] > 0
        msg.degree = int(np.degrees(
            np.arccos(np.dot(start_vector, end_vector))))

        self.topic_motor1.publish(msg)
        self.topic_motor2.publish(msg)


if __name__ == "__main__":
    Controller()
