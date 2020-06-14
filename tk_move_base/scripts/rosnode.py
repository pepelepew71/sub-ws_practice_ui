#!/usr/bin/env python

from __future__ import print_function

from math import radians

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty as EmptySrv
from tf.transformations import quaternion_from_euler
from tf import TransformListener, Transformer


class RosNode():

    def __init__(self):
        rospy.init_node(name='gui', anonymous=False)

        self.service_request_nomotion_update = None
        self.service_global_localization = None
        self.publisher_simple_goal = None

        self._setup_service()
        self._setup_publisher()
        self._setup_action()

    def _setup_service(self):
        # rospy.wait_for_service(service='/request_nomotion_update')
        self.service_request_nomotion_update = rospy.ServiceProxy(name='/request_nomotion_update', service_class=EmptySrv)

        # rospy.wait_for_service(service='/global_localization')
        self.service_global_localization = rospy.ServiceProxy(name='/global_localization', service_class=EmptySrv)

    def _setup_publisher(self):
        self.publisher_simple_goal = rospy.Publisher(name='/move_base_simple/goal', data_class=PoseStamped, queue_size=10)

    def subscribe_amcl_pose(self, callback):
        rospy.Subscriber(name="/amcl_pose", data_class=PoseWithCovarianceStamped, callback=callback)

    def subscribe_move_base_status(self, callback):
        rospy.Subscriber(name="/move_base/status", data_class=GoalStatusArray, callback=callback)

    def call_publisher(self, x_m, y_m, rz_deg):
        post = PoseStamped()
        post.header.frame_id = 'map'
        post.header.stamp = rospy.Time.now()
        post.pose.position.x = x_m
        post.pose.position.y = y_m
        q = quaternion_from_euler(0.0, 0.0, radians(rz_deg))
        post.pose.orientation.z = q[2]
        post.pose.orientation.w = q[3]
        self.publisher_simple_goal.publish(post)

    def _setup_action(self):
        self.action_move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.action_move_base.wait_for_server()

    def call_action_goal(self, x_m, y_m, rz_deg, feedback_cb, done_cb):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_m
        goal.target_pose.pose.position.y = y_m
        q = quaternion_from_euler(0.0, 0.0, radians(rz_deg))
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.action_move_base.send_goal(goal=goal, feedback_cb=feedback_cb, done_cb=done_cb)

    def call_action_cancel(self):
        try:
            self.action_move_base.cancel_all_goals()
        except Exception as e:
            print(e)
            rospy.loginfo("RosNode.call_action_cancel() failed")

    @classmethod
    def get_current_pose(self):
        pass
