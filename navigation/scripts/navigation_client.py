#!/usr/bin/env python3

import rospy
import actionlib
from navigation.msg import NavigateToAssemblyAction, NavigateToAssemblyGoal

class NavigationClient:
    """
    NavigationClient class interacts with the navigation server to navigate the humanoid robot to the assembly system.
    It sends a goal, receives continuous feedback on the progress, and retrieves the final result.
    """

    def __init__(self):
        """
        Initializes the NavigationClient and connects to the navigation server.
        """
        self.client = actionlib.SimpleActionClient('navigate_to_assembly', NavigateToAssemblyAction)
        rospy.loginfo("Waiting for navigation server...")
        self.client.wait_for_server()
        rospy.loginfo("Navigation server started.")

    def feedback_callback(self, feedback):
        """
        Callback function for receiving feedback from the navigation server.
        Prints the status updates.
        """
        rospy.loginfo(f"Navigation Status: {feedback.status}")

    def send_navigation_goal(self):
        """
        Sends the navigation goal to the server and handles feedback and result.
        """
        try:
            goal = NavigateToAssemblyGoal()
            # You can set any specific parameters related to the navigation goal here

            self.client.send_goal(goal, feedback_cb=self.feedback_callback)
            rospy.loginfo("Navigation Goal Sent.")

            # Wait for the result (blocking call)
            self.client.wait_for_result()

            # Retrieve the result
            result = self.client.get_result()

            if result.success:
                rospy.loginfo("Navigation completed successfully.")
            else:
                rospy.logerr(f"Navigation failed: {result.message}")

        except Exception as e:
            rospy.logerr(f"Error in sending navigation goal: {e}")

if __name__ == '__main__':
    rospy.init_node('navigation_client')
    client = NavigationClient()
    client.send_navigation_goal()
