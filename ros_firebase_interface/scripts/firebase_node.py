#!/usr/bin/env python3

import rospy
import firebase_admin
from firebase_admin import credentials

def initialise_firebase():
    """
    Initialize Firebase Admin SDK with the provided credentials.
    
    Returns:
        bool: True if initialization was successful, False otherwise.
    """
    try:
        cred_path = rospy.get_param('~firebase_credentials_path')
        cred = credentials.Certificate(cred_path)
        firebase_admin.initialize_app(cred)
        
        rospy.loginfo("Firebase app initialized.")
        return True
    except Exception as e:
        rospy.logerr("Failed to initialize Firebase app: %s", e)
        return False

if __name__ == '__main__':
    rospy.init_node('firebase_node')

    if not initialise_firebase():
        rospy.logerr("Failed to initialize Firebase app.")
        exit(1)
    
    rospy.spin()