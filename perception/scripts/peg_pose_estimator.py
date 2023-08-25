#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
from helpers import process_peg_pose_estimator_result
from perception.msg import PoseEstimationResult


class PegPoseEstimator:
    """
    ROS node for pose estimation of pegs in images using a trained PyTorch model.

    Subscribes to:
    - /ez_robot_camera/image_raw (sensor_msgs/Image): Raw images from the camera.

    Publishes to:
    - /peg_pose_estimator/result (std_msgs/String): Detection result as a string.
    - /peg_pose_estimator/image_with_boxes (sensor_msgs/Image): Image with bounding boxes around detected objects.

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
        self.pose_estimation_result_pub = rospy.Publisher("peg_pose_estimator/result", PoseEstimationResult, queue_size=1)
        self.image_with_boxes_pub = rospy.Publisher("peg_pose_estimator/image_with_inference", Image, queue_size=1)

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
            results = self.model.predict(preprocessed_image, conf=0.7, iou=0.7)  # results list

            # Since a single image is passed, the result will be a list of length 1
            result = results[0]

            im_array = result.plot()  # plot a BGR numpy array of predictions
            pose_estimation_result = process_peg_pose_estimator_result(result)

            rospy.loginfo(f'Detection Result: {pose_estimation_result}')
            rospy.loginfo(f'Detection Result: \n Box Data: {result.boxes.data}  \n Box Data xywh: {result.boxes.xywh} \n Keypoint Data: {result.keypoints.xy}')

            # Create the DetectionResult message object
            pose_estimation_result_msg = PoseEstimationResult()
            pose_estimation_result_msg.detected = pose_estimation_result['detected']
            pose_estimation_result_msg.bounding_box = pose_estimation_result['box_xywh']
            pose_estimation_result_msg.keypoint_1 = pose_estimation_result['keypoints_xy'][0]
            pose_estimation_result_msg.keypoint_2 = pose_estimation_result['keypoints_xy'][1]


            # Publish the detection result
            self.pose_estimation_result_pub.publish(pose_estimation_result_msg)
            # Publish the image with bounding boxes
            self.image_with_boxes_pub.publish(self.bridge.cv2_to_imgmsg(im_array, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == "__main__":
    rospy.init_node('peg_pose_estimator')
    detector = PegPoseEstimator()
    rospy.spin()
