# session_data_utils.py

import os
import rospy
from firebase_admin import firestore
from metrics_utils import capture_performance_metrics

import subprocess

def get_ros_nodes():
    """
    Retrieve the list of active ROS nodes.

    Returns:
        list: List of active ROS node names.
    """
    try:
        nodes = subprocess.check_output(["rosnode", "list"]).decode('utf-8').split('\n')
        return [node for node in nodes if node]
    except Exception as e:
        rospy.logerr(f"Error fetching ROS nodes: {e}")
        return []

def prepare_session_data():
    """
    Prepare the initial session data structure.

    Returns:
        dict: Session data structure.
    """
    session_data = {
        "start_time": firestore.SERVER_TIMESTAMP,
        "end_time": None,
        "nodes": get_ros_nodes(),
        "topics": [{"name": topic_name, "type": topic_type} for topic_name, topic_type in rospy.get_published_topics()],
        "system_info": {
            "ros_version": rospy.get_param('/rosdistro'),
            "machine_info": os.uname()[1],
            "env_variables": {
                "ROS_MASTER_URI": os.environ.get("ROS_MASTER_URI", "N/A")
            }
        },
        "errors": [],
        "warnings": [],
        "performance_metrics": capture_performance_metrics(),
        "session_active": True
    }
    return session_data

def prepare_end_session_data():
    """
    Prepare the session data structure for session end.

    Args:
        None

    Returns:
        dict: Session data structure for session end.
    """
    end_time = firestore.SERVER_TIMESTAMP

    session_data = {
        "end_time": end_time,
        "performance_metrics": capture_performance_metrics(),
        "session_active": False
    }
    return session_data
