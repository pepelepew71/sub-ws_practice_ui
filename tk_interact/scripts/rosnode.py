#!/usr/bin/env python

from __future__ import print_function

import rospy
import actionlib
import actionlib_tutorials.msg
from tk_interact.srv import Calculate
from std_msgs.msg import String


class RosNode():

    def __init__(self, parent):
        rospy.init_node(name='gui', anonymous=False)
        self.parent = parent

        self.service_calculate = None
        self.action_fibonacci = None

        self._subscribe_keyboard()
        self._setup_service_calculate()
        self._setup_action_fibonacci()

    def _subscribe_keyboard(self):
        rospy.Subscriber(name="/keyboard", data_class=String, callback=self._callback_keyboard)

    def _callback_keyboard(self, response):
        self.parent.frame_kb.callback_from_topic(response.data)

    def _setup_service_calculate(self):
        rospy.wait_for_service(service='/calculate')
        self.service_calculate = rospy.ServiceProxy(name='/calculate', service_class=Calculate)

    def call_service_calculate(self, expression):
        try:
            response = self.service_calculate(expression)
            return response.result
        except rospy.ServiceException as e:
            print(e)
            rospy.loginfo("RosNode.call_service_calculate() failed")

    def _setup_action_fibonacci(self):
        self.action_fibonacci = actionlib.SimpleActionClient('fibonacci_server', actionlib_tutorials.msg.FibonacciAction)
        self.action_fibonacci.wait_for_server()

    def call_action_goal(self, num, feedback_cb, done_cb):
        goal = actionlib_tutorials.msg.FibonacciGoal(order=num)
        self.action_fibonacci.send_goal(goal=goal, feedback_cb=feedback_cb, done_cb=done_cb)

    def call_action_cancel(self):
        try:
            self.action_fibonacci.cancel_all_goals()
        except Exception as e:
            print(e)
            rospy.loginfo("RosNode.call_action_cancel() failed")
