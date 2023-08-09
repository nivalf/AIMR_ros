#!/usr/bin/env python3

import rospy
import glob
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class ImageStreamer:
    """
    ROS Node: ImageStreamer class to read images from a directory and publish them as ROS messages.

    Attributes:
        image_paths (list): List of paths to image files in the directory.
        bridge (CvBridge): OpenCV to ROS bridge.
        image_pub (Publisher): ROS publisher for the image messages.
        rate (Rate): ROS rate for controlling the publish frequency.
    """
    
    def __init__(self, image_folder, topic_name='ez_robot_camera/image_raw', frame_rate=24):
        """
        Initialize ImageStreamer.

        Args:
            image_folder (str): Path to the folder containing the images.
            topic_name (str, optional): Name of the ROS topic to publish to. Defaults to 'ez_robot_camera/image_raw'.
            frame_rate (int, optional): Desired frame rate in frames per second. Defaults to 24.
        """
        self.image_paths = sorted(glob.glob(os.path.join(image_folder, '*.jpg')))
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
        self.rate = rospy.Rate(frame_rate)

    def stream_images(self):
        """Stream images from the directory, looping continuously."""
        image_index = 0
        while not rospy.is_shutdown():
            # Read the image
            image_path = self.image_paths[image_index]
            image = cv2.imread(image_path)

            # Convert the image to a ROS message
            image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")

            # Publish the image
            self.image_pub.publish(image_msg)

            # Increment the image index, looping back to the start if necessary
            image_index = (image_index + 1) % len(self.image_paths)

            # Sleep to achieve the desired frame rate
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('image_streamer')
    image_folder = os.path.expanduser('~/Dataset')
    streamer = ImageStreamer(image_folder)
    streamer.stream_images()
