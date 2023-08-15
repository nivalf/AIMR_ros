#!/usr/bin/env python3

import rospy
import actionlib
from navigation.msg import NavigateToAssemblyAction, NavigateToAssemblyFeedback, NavigateToAssemblyResult
from perception.msg import DetectionResult
# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

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
        self.server = actionlib.SimpleActionServer('tb3_navigate_to_assembly', NavigateToAssemblyAction, self.execute_cb, False)
        self.server.start()
        self.state = self.SCANNING

        self.detection_subscriber = rospy.Subscriber('/assembly_system_detector/result', DetectionResult, self.resultdetection_cb)
        self.detection_count = 0

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        self.stop_distance = rospy.get_param('~stop_distance', 0.5)

    def resultdetection_cb(self, msg):
        """
        Callback for assembly system detection. Updates FSM state based on detection status.
        """
        rospy.loginfo(f"Detection result: {msg}")
        if msg.detected == True:
            if self.state == self.SCANNING:
                self.set_state(self.MOVING_FORWARD)
            elif self.state == self.MOVING_FORWARD and msg.percentage_cover > 75:
                self.set_state(self.STOPPED)
            self.detection_count = 0
        else:
            self.detection_count += 1
            if self.state == self.MOVING_FORWARD and self.detection_count >= 5:
                self.set_state(self.SCANNING)

    def execute_cb(self, goal):
        """
        Execution logic for the navigation action using FSM.
        Implements navigation process and sends feedback.
        """
        feedback = NavigateToAssemblyFeedback()
        result = NavigateToAssemblyResult()

        self.set_state(self.SCANNING)

        try:
            while not self.server.is_preempt_requested() and not rospy.is_shutdown():
                # check collision
                if self.tb3_lidar.min_distance < self.stop_distance:
                    rospy.loginfo(f"Potential Collision detected at {self.tb3_lidar.min_distance} m.")
                    self.set_state(self.STOPPED)
                    break

                if self.state == self.SCANNING:
                    feedback.status = "Scanning for assembly system."
                    # publish a velocity command to make the robot scanning. velocity set in transition fn.
                    self.vel_controller.publish()

                elif self.state == self.MOVING_FORWARD:
                    feedback.status = "Assembly system detected. Moving forward."
                    # publish a velocity command to make the robot move. velocity set in transition fn.
                    self.vel_controller.publish()

                elif self.state == self.STOPPED:
                    feedback.status = "Navigation stopped."
                    # publish a velocity command to make the robot stop. velocity set in transition fn.
                    self.vel_controller.publish()
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

    def set_state(self, state):
        """
        Sets the state of the FSM.
        Transition function: Add logic that should be 
        executed only at state transitions here.
        """
        if state == self.SCANNING:
            # set the robot's angular velocity...
            self.vel_controller.set_move_cmd(0, 1.5)

        elif state == self.MOVING_FORWARD:
            self.vel_controller.set_move_cmd(0.2, 0)

        elif state == self.STOPPED:
            self.vel_controller.set_move_cmd(0, 0)

        self.state = state

if __name__ == '__main__':
    rospy.init_node('navigation_server')
    server = NavigationServer()
    rospy.spin()
