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
from yolo_step1.srv import PersonDetection, PersonDetectionResponse


class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()

        # self.image_res = 240, 320, 3  # Camera resolution: height, width
        self.image_res = rospy.get_param("image_processor/height"), rospy.get_param("image_processor/width"), 3  # Camera resolution: height, width
                         
        # The numpy array to pour the image data into
        self.image_np = np.zeros(self.image_res)

        # Subscribe on your robot's camera topic
        # NOTE: Make sure you use the provided listener for this subscription
        camera = rospy.get_param("image_processor/camera")
        self.camera_subscriber = rospy.Subscriber(
            camera, Image, self.camera_listener)

        # Instantiate your YOLO object detector/classifier model
        file_address = rospy.get_param("image_processor/model")
        self.model: YOLO = YOLO(file_address)

        # You need to update results each time you call your model
        self.results: Results = None
        self.objects = {}

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # Setup "human detection" service
        rospy.Service('/detection', PersonDetection, self.detection)
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
                self.results = self.model(self.image_np, verbose=False)

                annotator = Annotator(frame, line_width=1, font_size=1)
                self.objects = {}

                for res in self.results:
                    try:
                        if res.names is None or res.boxes.xyxy is None:
                            continue
                        box = res.boxes.xyxy[0]
                        label = res.names[0]
                        annotator.box_label(box, label=label)

                        if label:
                            self.objects[label] = box.tolist()

                    except Exception:
                        pass

                frame = annotator.result()

                self.is_started = True

                image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("robot_view", image)
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass

    def detection(self, request: PersonDetection) -> PersonDetectionResponse:
        # Set detected to True if a human is detected
        label = request.label
        response = PersonDetectionResponse()
        box = self.objects.get(label)
        if box:
            response.x1 = box[0]
            response.y1 = box[1]
            response.x2 = box[2]
            response.y2 = box[3]
        elif self.is_started:
            response.x1 = 0
            response.y1 = 0
            response.x2 = 0
            response.y2 = 100
        else:
            response.x1 = 0
            response.y1 = 0
            response.x2 = 320
            response.y2 = 100
        return response


if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()
