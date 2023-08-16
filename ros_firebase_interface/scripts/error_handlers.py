# error_handlers.py

import rospy

def handle_firebase_error(e, action="perform operation"):
    """
    Log a Firebase-specific error.

    Args:
        e (Exception): Caught exception.
        action (str): Description of the action that caused the error.
    """
    rospy.logerr(f"Failed to {action} in Firebase: {e}")
