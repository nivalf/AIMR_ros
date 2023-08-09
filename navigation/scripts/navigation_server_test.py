#!/usr/bin/env python3

import rospy
import actionlib
from std_msgs.msg import String
from navigation.msg import NavigateToAssemblyAction, NavigateToAssemblyFeedback, NavigateToAssemblyResult
from ez_robot_interface.msg import SendEZCommandGoal, SendEZCommandAction

class NavigationServer:
    """
    NavigationServer class using Finite State Machine (FSM) to manage the navigation of the humanoid robot.
    States:
        SCANNING: Turning left, looking for the assembly system.
        MOVING_FORWARD: Moving towards the assembly system.
        STOPPED: Stopped.
    """

    SCANNING = 0
    MOVING_FORWARD = 1
    STOPPED = 2

    def __init__(self):
        """
        Initializes the NavigationServer and sets up the FSM logic.
        """
        # self.server = actionlib.SimpleActionServer('navigate_to_assembly', NavigateToAssemblyAction, self.execute, False)
        # self.server.start()

        self.ez_command_client = actionlib.SimpleActionClient('sendEZCommand', SendEZCommandAction)
        self.ez_command_client.wait_for_server()

        rospy.loginfo(" Sending command to turn left")
        self.send_ez_command('Left()')

        # self.detection_subscriber = rospy.Subscriber('/assembly_system_detector/results', String, self.detection_callback)
        # self.state = self.SCANNING
        # self.detection_count = 0


    def send_ez_command(self, command):
        """
        Sends a command to the sendEZCommand server.
        """
        goal = SendEZCommandGoal()
        goal.command = command
        self.ez_command_client.send_goal(goal)
        self.ez_command_client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('navigation_server')
    server = NavigationServer()
    rospy.spin()
