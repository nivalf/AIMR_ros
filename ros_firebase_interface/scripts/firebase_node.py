#!/usr/bin/env python3

# firebase_node.py

import rospy
from firebase_handler import FirebaseHandler
from session_data_utils import prepare_session_data, prepare_end_session_data
from error_handlers import handle_firebase_error

class FirebaseNode:
    """
    ROS node for interfacing with Firebase.
    Manages the session data capture and storage in Firebase.
    """

    def __init__(self):
        """
        Initialize the FirebaseNode instance and ROS node.
        """
        rospy.init_node("firebase_node")

        # Initialize Firebase
        cred_path = rospy.get_param('~firebase_credentials_path')
        self.firebase = FirebaseHandler(cred_path)

        rospy.loginfo("Firebase initialized.")

        self.doc_ref = None

        # Register ROS shutdown hook
        rospy.on_shutdown(self.end_session)

    def start_session(self):
        """
        Start a new ROS session and save its initial data structure to Firebase.

        Returns:
            DocumentReference: Reference to the Firestore document of the session.
        """
        session_data = prepare_session_data()

        # Push the session data to Firebase
        try:
            self.doc_ref = self.firebase.add_document('ros_sessions', session_data)
            rospy.loginfo(f"Session started with ID: {self.doc_ref}")
            return self.doc_ref
        except Exception as e:
            handle_firebase_error(e, action="start session")
            return None

    def end_session(self):
        """
        Mark the end of the ROS session and save changes to Firebase.
        """
        if not self.doc_ref:
            rospy.logerr("No active session found to end.")
            return

        session_data = prepare_end_session_data()

        try:
            self.firebase.update_document(self.doc_ref, session_data)
        except Exception as e:
            handle_firebase_error(e, action="end session")

if __name__ == "__main__":
    node = FirebaseNode()
    node.start_session()
    rospy.spin()
