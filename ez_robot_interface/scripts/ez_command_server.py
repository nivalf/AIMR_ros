#!/usr/bin/env python3

import rospy
import actionlib
import socket
from ez_robot_interface.msg import SendEZCommandAction, SendEZCommandResult

class EZCommandServer:
    """
    A ROS Action Server that serves as an interface to a EZ Robot's 'TCP Sript Server Raw' skill in ARC.

    This Action Server connects to a specified TCP server on initialization,
    and listens for goals containing string commands. It sends these commands
    to the TCP server, receives responses, and sends the responses back as the
    results of the action.

    :param ip: The IP address of the TCP server.
    :param port: The port number of the TCP server.
    :param max_retries: Maximum number of connection retries to the TCP server. (Optional)

    Usage:
        server = EZCommandServer('192.168.0.11', 8080)
    """

    def __init__(self, ip, port, max_retries=5):
        self.server = actionlib.SimpleActionServer('sendEZCommand', SendEZCommandAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()
        rospy.on_shutdown(self.on_shutdown)
        
        self.s = None
        self.ip = ip
        self.port = port
        self.max_retries = max_retries
        
        self.connect_to_tcp_server()

        rospy.loginfo("The 'Send EZ Command Action Server' is active...")

    def connect_to_tcp_server(self):
        """Attempts to connect to the TCP server with retries."""

        rospy.loginfo("Attempting connection...")
        retry_count = 0
        while retry_count < self.max_retries and not rospy.is_shutdown():
            try:
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.s.connect((self.ip, self.port))
                rospy.loginfo("Connected to TCP Server")
                return
            except Exception as e:
                rospy.logwarn(f"Failed to connect to TCP Server. Attempt {retry_count + 1}/{self.max_retries}. Error: {e}")
                retry_count += 1
                self.s = None
                rospy.sleep(5)

        rospy.logerr("Failed to connect to TCP Server after maximum retries")
        rospy.signal_shutdown("Could not connect to TCP Server")

    def execute_cb(self, goal):
        """Executes the action by sending the command to the TCP server and returning the response."""
        command = goal.command + '\r\n'
        rospy.loginfo(f"Sending command: {command}")

        if self.s is None:
            rospy.logerr("TCP connection is not established. Cannot send command")
            self.server.set_aborted()
            return

        try:
            self.s.sendall(command.encode())
            response = self.s.recv(1024).decode()
        except Exception as e:
            rospy.logerr(f"Error communicating with TCP Server: {e}")
            self.server.set_aborted()
            return

        rospy.loginfo(f"Received response: {response}")
        result = SendEZCommandResult()
        result.response = response
        self.server.set_succeeded(result)

    def on_shutdown(self):
        """Closes the TCP connection on node shutdown."""
        rospy.loginfo("Disconnecting from TCP Server")
        if self.s is not None:
            self.s.close()

if __name__ == '__main__':
    # ARC_Host_IP = rospy.get_param('~ARC_Host_IP')
    # ARC_Host_Port = rospy.get_param('~ARC_Host_Port')

    rospy.init_node('ez_command_action_server')
    # server = EZCommandServer(ARC_Host_IP, ARC_Host_Port)
    server = EZCommandServer('192.168.0.11', 8080)
    rospy.spin()
