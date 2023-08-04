#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class ImageDatasetCreator:
    """
    Node: image_dataset_creator

    Description:
    This node subscribes to the ez_robot_camera/image_raw topic and saves the incoming images to a specified directory 
    to create a dataset.

    Parameters:
    ~save_directory (str, default: /tmp/dataset): Directory where the images will be saved.

    Subscribed Topics:
    ez_robot_camera/image_raw (sensor_msgs/Image): Camera images that are to be saved to the dataset directory.
    """

    def __init__(self):
        """
        Initialize the image dataset creator node.

        Sets up parameters, directory checks, initializes the image counter, sets up CvBridge, and subscribes 
        to the camera topic.
        """
        
        # Initialize the ROS node
        rospy.init_node('image_dataset_creator', anonymous=True)

        # Directory to save the images
        self.save_dir = os.path.expanduser(rospy.get_param('~save_directory', '~/Dataset'))
        
        # Check if the directory exists, if not, create it
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # Image counter to save images with unique names
        self.img_counter = 0

        # Bridge to convert ROS Image type to OpenCV Image type
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber('ez_robot_camera/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        """
        Callback function for the camera topic subscription.

        Args:
        - data (sensor_msgs/Image): The image message received from the topic.

        This function saves the received image to the specified directory with a unique name.
        """
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        img_name = os.path.join(self.save_dir, f'image_{self.img_counter}.jpg')
        cv2.imwrite(img_name, cv_image)
        rospy.loginfo(f"Saved {img_name}")
        self.img_counter += 1

if __name__ == '__main__':
    try:
        ImageDatasetCreator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
