#!/usr/bin/env python3

import rospy
import torch
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from torchvision import transforms
from ultralytics import YOLO
from PIL import Image as PILImage

class AssemblySystemDetector:
    """
    ROS node for detecting assembly systems in images using a trained PyTorch model.

    Subscribes to:
    - /ez_robot_camera/image_raw (sensor_msgs/Image): Raw images from the camera.

    Publishes to:
    - /assembly_system_detector/results (std_msgs/String): Detection results as a string.
    - /assembly_system_detector/image_with_boxes (sensor_msgs/Image): Image with bounding boxes around detected objects.

    """

    def __init__(self):
        # Load the PyTorch model
        model_path = rospy.get_param('~model_path')
        self.model = YOLO(model_path)

        # Image conversion bridge
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber("ez_robot_camera/image_raw", Image, self.callback)

        # Publishers for the detection result and image with bounding boxes
        self.detection_result_pub = rospy.Publisher("assembly_system_detector/results", String, queue_size=10)
        self.image_with_boxes_pub = rospy.Publisher("assembly_system_detector/image_with_inference", Image, queue_size=10)

    def preprocess_image(self, image):
        """Resize the image to the required dimensions (320x256) and normalize."""
        image = cv2.resize(image, (320, 256))
        return image

    def callback(self, data):
        try:
            # Convert the ROS image message to a NumPy array
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Preprocess the image
            preprocessed_image = self.preprocess_image(cv_image)

            # Run inference 
            results = self.model(preprocessed_image)  # results list

            # Show the results
            for r in results:
                im_array = r.plot()  # plot a BGR numpy array of predictions
                # image_with_boxes = PILImage.fromarray(im_array[..., ::-1])  # RGB PIL image
                # image_with_boxes.show()

            # Publish the detection result
            self.detection_result_pub.publish(String(data="detection result"))
            # Publish the image with bounding boxes
            self.image_with_boxes_pub.publish(self.bridge.cv2_to_imgmsg(im_array, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == "__main__":
    rospy.init_node('assembly_system_detector')
    detector = AssemblySystemDetector()
    rospy.spin()
