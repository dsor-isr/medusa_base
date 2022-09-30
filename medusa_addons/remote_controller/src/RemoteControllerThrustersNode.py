#!/usr/bin/env python

"""
@author: Marcelo Fialho Jacinto
@email: marcelo.jacinto@tecnico.ulisboa.pt
@date: 25/11/2021
@licence: MIT
"""
import rospy
import pygame
from std_msgs.msg import Float64
from auv_msgs.msg import NavigationStatus
from dsor_msgs.msg import Thruster
from joystick_controller import ControlAssignment


class RemoteControllerNode:
    """
    Remote controller ROS node class. Receives from a joystick (using pygame) the desired controls for
    the inner-loops and publishes these periodically (at a pre-defined frequency) to the inner-loops of the
    vehicle
    """

    def __init__(self):
        """
        Class constructor. Initializes the ros node, loads the parameters from the ros parameter server, creates the inner-loops
        publishers and initializes the timer that publishes the desired inputs to the inner-loops
        """

        # ---Initialize the ROS NODE---
        rospy.init_node('remote_controller_node')
        self.node_frequency = rospy.get_param('~node_frequency', 10)
        self.h_timerActivate = False

        # ---Load the ROS configurations---
        self.initializePublishers()
        self.initializeSubscribers()

        # ---Initialize the pygame to read the inputs from the joystick
        self.initializeJoystick()

        # --- Save relevant state variables from the AUV vehicle
        self.yaw_state_ = 0.0 # in (deg)

        # ---Start the ROS NODE callback---
        self.initializeTimer()

    def timerCallback(self, event):
        """
        Callback used to publish to the inner-loops the desired control inputs
        :param event: A timer event - unused but required by the ROS API
        """

        # Check for the desired inputs for the inner-loops
        desired_inputs = self.control_assignment.check_events()

        # Log to the screen the desired inputs
        rospy.loginfo(desired_inputs)

        # Publish the linear speed references
        self.thruster_port_pub.publish(desired_inputs["thruster_port"])
        self.thruster_stbd_pub.publish(desired_inputs["thruster_stbd"])
        #self.thruster_both_pub.publish(desired_inputs["thruster_both"])

    def initializeTimer(self):
        """
        Method that starts the system timer that periodically calls a callback
        """
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def initializeSubscribers(self):
        """
        Method that initializes the ROS subscribers (to receive the current state of the AUV)
        """
        rospy.loginfo('Initializing Subscribers for RemoteControllerNode')

        # Define the state subscriber
        self.state_sub = rospy.Subscriber(rospy.get_param('~topics/subscribers/state'), NavigationStatus, self.state_callback, queue_size=1)

    def initializePublishers(self):
        """
        Method that initializes the ROS publishers (to publish the references for the inner-loops)
        """
        rospy.loginfo('Initializing Publishers for RemoteControllerNode')

        # Defining the publishers for linear speed references
        self.thruster_port_pub = rospy.Publisher(rospy.get_param('~topics/publishers/thruster_port'), Float64, queue_size=1)
        self.thruster_stbd_pub = rospy.Publisher(rospy.get_param('~topics/publishers/thruster_stbd'), Float64, queue_size=1)
        #self.thruster_both_pub = rospy.Publisher(rospy.get_param('~topics/publishers/thruster_both'), Thruster, queue_size=1)

    def initializeJoystick(self):
        """
        Method that initializes the joystick driver (using pygame)
        """

        # Create the joystick object
        try:
            # Get the controller configurations from the parameter server and save them in a dictionary
            controller_configurations = {"thruster_port": rospy.get_param('~button_assignment/thruster_port'),
                                         "thruster_stbd": rospy.get_param('~button_assignment/thruster_stbd'),
                                         "thruster_both": rospy.get_param('~button_assignment/thruster_both')}

            # Initialize the main joystick and bind key assignments according to configuration in ros parameter server
            self.control_assignment = ControlAssignment(controller_configurations)

        except pygame.error as e:
            # Check if we are able to create the joystick object. If not, terminate the node and send an error message
            rospy.logerr("Could not start joystick driver: " + str(e))
            rospy.signal_shutdown("")
        except ValueError as e:
            # If there is an error in keybindings, inform the user
            rospy.logerr(e)
            rospy.signal_shutdown("Invalid joystick bindings")

    def state_callback(self, msg):
        """
        Callback that is called when a message with the current state of the AUV is received.
        Currently only the yaw orientation is saved (used to switch between yaw and yaw-rate controllers)
        :param msg: NavigationStatus message
        """
        self.yaw_state_ = float(msg.orientation.z)

def main():
    """
    Initialize the RemoteControllerNode and let the timer callback do all the work
    """
    remote_controller = RemoteControllerNode()
    rospy.spin()


if __name__ == '__main__':
    main()
