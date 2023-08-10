#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import requests
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraStreamer:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("ez_robot_camera/image_raw", Image, queue_size=1)
        self.camera_url = "http://192.168.0.11/CameraImage.jpg?password=admin&c=Camera"

    def fetch_and_publish(self):
        try:
            response = requests.get(self.camera_url, stream=True, timeout=0.5)
            response.raise_for_status()
            
            # Convert the response content to a numpy array
            np_arr = np.frombuffer(response.content, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Ensure the image is not empty or None before converting
            if image_np is not None and image_np.size > 0:
                image_msg = self.bridge.cv2_to_imgmsg(image_np, "bgr8")
                self.image_pub.publish(image_msg)
            else:
                rospy.logwarn("Received an empty image.")
                rospy.loginfo(f"Content type: {response.headers.get('content-type')}, Content length: {len(response.content)}")
        except Exception as e:
            rospy.logerr(f"Error fetching image: {e}")


    def run(self):
        rate = rospy.Rate(5)  # 10 Hz
        while not rospy.is_shutdown():
            self.fetch_and_publish()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ez_camera_streamer', anonymous=True)
    streamer = CameraStreamer()
    streamer.run()
