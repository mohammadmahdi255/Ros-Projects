#!/home/mahdi255/.pyenv/shims/python

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results


# ROS
import rospy
from sensor_msgs.msg import Image
from yolo_step2.srv import WallDetection, WallDetectionResponse


class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()

        # self.image_res = 240, 320, 3  # Camera resolution: height, width
        self.image_res = rospy.get_param("image_processor/height"), rospy.get_param(
            "image_processor/width"), 3  # Camera resolution: height, width

        # The numpy array to pour the image data into
        self.image_np = np.zeros(self.image_res)

        # Subscribe on your robot's camera topic
        # NOTE: Make sure you use the provided listener for this subscription
        camera = rospy.get_param("image_processor/camera")
        self.camera_subscriber = rospy.Subscriber(
            camera, Image, self.camera_listener)

        # You need to update results each time you call your model
        self.results = None

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        self.is_started = False

        self.update_view()

    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)

    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0:  # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(
                    self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                frame = copy.deepcopy(self.image_np)
                frame[abs(frame - 170) < 3] = 102

                # Define the two rectangles
                rect1 = [(0, 700), (400, 1080)]
                rect2 = [(1520, 700), (1920, 1080)]

                # Extract the pixels within the two rectangles
                rect1_pixels = frame[rect1[0][1]:rect1[1]
                                     [1], rect1[0][0]:rect1[1][0]]
                rect2_pixels = frame[rect2[0][1]:rect2[1]
                                     [1], rect2[0][0]:rect2[1][0]]

                # Extract the R channel of the pixels
                rect1_r = rect1_pixels[:, :, 0]
                rect2_r = rect2_pixels[:, :, 0]

                weights1 = np.where(abs(rect1_r - 152) <= 2, 1, 3)
                weights2 = np.where(abs(rect2_r - 152) <= 2, 1, 3)

                self.results = (np.average(rect1_r, weights=weights1),
                                np.average(rect2_r, weights=weights2))

                # Print the average R value of each rectangle
                if not self.is_started:
                    print("Rect 1 R value: ", self.results[0])
                    print("Rect 1 R value: ", np.mean(rect1_r))
                    print("Rect 2 R value: ", self.results[1])
                    rospy.Service('/detection_wall',
                                  WallDetection, self.detection_wall)

                self.is_started = True

                image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Draw the two rectangles on the image
                rect_color = (0, 0, 255)
                cv2.rectangle(
                    image, rect1[0], rect1[1], rect_color, thickness=3, lineType=cv2.LINE_8)
                cv2.rectangle(
                    image, rect2[0], rect2[1], rect_color, thickness=3, lineType=cv2.LINE_8)

                cv2.imshow("robot_view", image)
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass

    def detection_wall(self, request: WallDetection) -> WallDetectionResponse:
        response = WallDetectionResponse()

        if self.results:
            response.left = self.results[0] < 150
            response.right = self.results[1] < 150

        return response


if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()
