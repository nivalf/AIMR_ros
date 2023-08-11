#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
from helpers import process_result
from perception.msg import DetectionResult


class AssemblySystemDetector:
    """
    ROS node for detecting assembly systems in images using a trained PyTorch model.

    Subscribes to:
    - camera/color/image_raw" (sensor_msgs/Image): Raw images from the camera of turtlebot.

    Publishes to:
    - /assembly_system_detector/result (std_msgs/String): Detection result as a string.
    - /assembly_system_detector/image_with_boxes (sensor_msgs/Image): Image with bounding boxes around detected objects.

    """

    def __init__(self):
        # Load the PyTorch model
        model_path = rospy.get_param('~model_path')
        self.model = YOLO(model_path)

        # Image conversion bridge
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber("camera/color/image_raw", Image, self.callback)

        # Publishers for the detection result and image with bounding boxes
        self.detection_result_pub = rospy.Publisher("assembly_system_detector/result", DetectionResult, queue_size=1)
        self.image_with_boxes_pub = rospy.Publisher("assembly_system_detector/image_with_inference", Image, queue_size=1)

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
            # Set confidence threshold (default 0.25) and IoU threshold (default 0.7)
            results = self.model.predict(preprocessed_image, conf=0.8, iou=0.7)  # results list

            # Since a single image is passed, the result will be a list of length 1
            result = results[0]

            im_array = result.plot()  # plot a BGR numpy array of predictions
            detection_result = process_result(result)

            rospy.loginfo(f'Detection Result: \n {result.boxes.data}')

            # Create the DetectionResult message object
            detection_result_msg = DetectionResult()
            detection_result_msg.detected = detection_result['detected']
            detection_result_msg.max_area = detection_result['max_area']
            detection_result_msg.num_boxes = detection_result['num_boxes']
            detection_result_msg.confidence = detection_result['confidence']
            detection_result_msg.percentage_cover = detection_result['percentage_cover']


            # Publish the detection result
            self.detection_result_pub.publish(detection_result_msg)
            # Publish the image with bounding boxes
            self.image_with_boxes_pub.publish(self.bridge.cv2_to_imgmsg(im_array, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == "__main__":
    rospy.init_node('assembly_system_detector')
    detector = AssemblySystemDetector()
    rospy.spin()
