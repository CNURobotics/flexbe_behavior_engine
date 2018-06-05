#!/usr/bin/env python

import roslib; roslib.load_manifest('flexbe_core')
import rospy
import actionlib
from threading import Timer
import time
import sys, traceback
import os

traceback_template = '''Traceback (most recent call last):
  File "%(filename)s", line %(lineno)s, in %(name)s
%(type)s: %(message)s\n''' # Skipping the "actual line" item

from flexbe_core.logger import Logger


class ProxyActionClient(object):
    """
    A proxy for calling actions.
    """
    _clients = {}

    _msg_types = {}
    _result = {}
    _feedback = {}

    def __init__(self, topics = {}, enter_wait_duration=0.0):
        """
        Initializes the proxy with optionally a given set of clients.

        @type topics              : dictionary
        @param topics             : A dictionary containing a collection of topic - message type pairs.
        @type wait_duration       : float
        @param enter_wait_duration: Defines how long to wait for the given client if it is not available right now.
        """

        # Try to connect on startup
        wait_duration = 10 # default on startup
        if (enter_wait_duration > 0):
            wait_duration = enter_wait_duration

        for topic, msg_type in topics.iteritems():
            self._msg_types[topic] = msg_type
            self.setupClient(topic, msg_type, wait_duration)

    def setupClient(self, topic, msg_type, wait_duration=10.):
        """
        Tries to set up an action client for calling it later.

        @type topic: string
        @param topic: The topic of the action to call.

        @type msg_type: msg type
        @param msg_type: The type of messages of this action client.

        @type wait_duration: float
        @param wait_duration: Defines how long to wait for the given client if it is not available right now.
        """
        if topic not in ProxyActionClient._clients:
            client = actionlib.SimpleActionClient(topic, msg_type)

            # Register this client with proxy
            ProxyActionClient._clients[topic] = client

            # Attempt to validate the connection
            t = Timer(1, self._print_wait_warning, [topic])
            t.start()
            available = client.wait_for_server(rospy.Duration.from_sec(wait_duration))
            warning_sent = False
            try:
                t.cancel()
            except Exception as ve:
                # already printed the warning
                warning_sent = True

            if not available:
                Logger.logerr("Action client %s timed out before connection made!" % topic)
            else:
                if warning_sent:
                    Logger.loginfo("Finally found action client %s..." % (topic))


    def send_goal(self, topic, goal):
        """
        Performs an action call on the given topic.

        @type topic: string
        @param topic: The topic to call.

        @type goal: action goal
        @param goal: The request to send to the action server.
        """
        if topic not in ProxyActionClient._clients:
            raise ValueError('ProxyActionClient - topic %s not yet registered!' % topic)

        # Verify that there is an action server ready to process the goal
        if not ProxyActionClient._clients[topic].wait_for_server(rospy.Duration.from_sec(0.0001)):
            raise ValueError('ProxyActionClient - topic %s is not yet connected to server!' % topic)

        # Clear prior results
        ProxyActionClient._result[topic] = None
        ProxyActionClient._feedback[topic] = None

        ProxyActionClient._clients[topic].send_goal(goal,
            done_cb = lambda ts, r: self._done_callback(topic, ts, r),
            feedback_cb = lambda f: self._feedback_callback(topic, f)
        )

    def _done_callback(self, topic, terminal_state, result):
        ProxyActionClient._result[topic] = result

    def _feedback_callback(self, topic, feedback):
        ProxyActionClient._feedback[topic] = feedback


    def is_available(self, topic):
        """
        Checks if the client on the given action topic is available.

        @type topic: string
        @param topic: The topic of interest.
        """
        return topic in ProxyActionClient._clients

    def has_result(self, topic):
        """
        Checks if the given action call already has a result.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._result[topic] is not None

    def get_goal_status_text(self, topic):
        """
        Returns the result status text if any.

        @type topic: string
        @param topic: The topic of interest.
        """

        if (not topic in ProxyActionClient._clients):
            return "Failed to get status text: Invalid action topic = [" + topic +"]"

        try:
            status = ProxyActionClient._clients[topic].get_goal_status_text()
            return status
        except:
            return "Failed to retrieve goal status text for topic = [" + topic +"]"

    def get_result(self, topic):
        """
        Returns the result message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._result[topic]

    def has_feedback(self, topic):
        """
        Checks if the given action call has any feedback.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._feedback[topic] is not None

    def get_feedback(self, topic):
        """
        Returns the latest feedback message of the given action call.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._feedback[topic]

    def get_state(self, topic):
        """
        Determines the current actionlib state of the given action topic.
        A list of possible states is defined in actionlib_msgs/GoalStatus.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._clients[topic].get_state()

    def is_active(self, topic):
        """
        Determines if an action request is already being processed on the given topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        return ProxyActionClient._clients[topic].simple_state != actionlib.SimpleGoalState.DONE

    def cancel(self, topic):
        """
        Cancels the current action call on the given action topic.

        @type topic: string
        @param topic: The topic of interest.
        """
        ProxyActionClient._clients[topic].cancel_goal()

    def _print_wait_warning(self, topic):
        Logger.logwarn("Waiting for action client %s..." % (topic))
