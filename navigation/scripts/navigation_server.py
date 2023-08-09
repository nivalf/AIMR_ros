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
        self.server = actionlib.SimpleActionServer('navigate_to_assembly', NavigateToAssemblyAction, self.execute, False)
        self.server.start()

        self.ez_command_client = actionlib.SimpleActionClient('sendEZCommand', SendEZCommandAction)
        self.ez_command_client.wait_for_server()

        self.detection_subscriber = rospy.Subscriber('/assembly_system_detector/results', String, self.detection_callback)
        self.state = self.SCANNING
        self.detection_count = 0

    def detection_callback(self, msg):
        """
        Callback for assembly system detection. Updates FSM state based on detection status.
        """
        if msg.data == 'detected':
            if self.state == self.SCANNING:
                self.state = self.MOVING_FORWARD
            self.detection_count = 0
        else:
            self.detection_count += 1
            if self.state == self.MOVING_FORWARD and self.detection_count >= 5:
                self.state = self.SCANNING

    def execute(self, goal):
        """
        Execution logic for the navigation action using FSM.
        Implements navigation process and sends feedback.
        """
        feedback = NavigateToAssemblyFeedback()
        result = NavigateToAssemblyResult()

        try:
            while not self.server.is_preempt_requested() and not rospy.is_shutdown():
                if self.state == self.SCANNING:
                    feedback.status = "Scanning for assembly system."
                    self.send_ez_command('SayEZB("Scanning for assembly system.")')
                    self.send_ez_command('Left()')

                elif self.state == self.MOVING_FORWARD:
                    feedback.status = "Assembly system detected. Moving forward."
                    self.send_ez_command('Forward()')

                elif self.state == self.STOPPED:
                    feedback.status = "Navigation stopped."
                    self.send_ez_command('Stop()')
                    break

                self.server.publish_feedback(feedback)
                rospy.sleep(0.1) # Polling interval

            result.success = True
            result.message = "Navigation complete."
            self.server.set_succeeded(result)

        except Exception as e:
            rospy.logerr(f"Error in navigation process: {e}")
            result.success = False
            result.message = "Navigation failed due to an error."
            self.server.set_aborted(result)

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
