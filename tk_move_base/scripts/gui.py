#!/usr/bin/env python

from __future__ import print_function

from collections import OrderedDict
from math import radians, degrees, sqrt
import os
import Tkinter as tk
import ttk

import rospy
from tf.transformations import euler_from_quaternion

from rosnode import RosNode

PATH = os.path.dirname(os.path.abspath(__file__))
COLOR_GRAY = "#d9d9d9"
COLOR_GRAY2 = "#999999"
DIMS = ('x', 'y', 'rz')
ICONS = dict()

def set_ICONS():
    global ICONS
    ICONS['pause'] = tk.PhotoImage(file=PATH + '/icons/' + 'outline_pause_black_18dp.png')
    ICONS['play'] = tk.PhotoImage(file=PATH + '/icons/' + 'outline_play_arrow_black_18dp.png')
    ICONS['stop'] = tk.PhotoImage(file=PATH + '/icons/' + 'outline_stop_black_18dp.png')
    ICONS['trend_up'] = tk.PhotoImage(file=PATH + '/icons/' + 'outline_trending_up_black_18dp.png')

def set_style():
    s = ttk.Style()
    s.configure('lock.Toolbutton', background=COLOR_GRAY2, width=36, height=36)
    s.map('lock.Toolbutton', background=[('selected', 'white')])


class MainWindow(tk.Tk, object):

    def __init__(self):
        super(MainWindow, self).__init__()
        set_ICONS()
        set_style()

        # self.geometry("470x300")
        self.resizable(width=False, height=False)
        self.title("tk move_base")

        self.labelframe_monitor = LabelFrameRobotMonitor(master=self)
        self.labelframe_move_base = LabelFrameMoveBase(master=self)
        self.labelframe_amcl = LabelFrameAMCL(master=self)

        self._pack_widgets()
        self._setup_bind()

    def _pack_widgets(self):
        self.labelframe_monitor.pack(side='top', fill='x', padx=5, pady=5, expand=False)
        self.labelframe_move_base.pack(side='top', fill='x', padx=5, expand=False)
        self.labelframe_amcl.pack(side='top', fill='x', padx=5, pady=5, expand=False)

    def _setup_bind(self):
        self.bind('<Escape>', lambda evt: self.destroy())


class LabelFrameRobotMonitor(tk.LabelFrame, object):

    def __init__(self, master):
        super(LabelFrameRobotMonitor, self).__init__(master=master, text="Robot Monitor")

        self._start_pose = None
        self._goal_pose = None

        self.state_labs = OrderedDict()  # self.state_rbs = OrderedDict()
        self.pose_ents = OrderedDict()
        self.pro = None

        self._setup_widgets()

        ROS_NODE.subscribe_amcl_pose(callback=self.update_pose_ents)
        ROS_NODE.subscribe_move_base_status(callback=self.update_state)

    def _setup_widgets(self):
        tk.Label(master=self, text="State").grid(row=0, column=0, columnspan=2)
        tk.Label(master=self, text="x").grid(row=0, column=3)
        tk.Label(master=self, text="y").grid(row=0, column=4)
        tk.Label(master=self, text="rz").grid(row=0, column=5)
        tk.Label(master=self, text="Progress").grid(row=0, column=7)

        self._setup_state_labs()  # self._setup_state_rbs()
        self._setup_ents()
        self._setup_progress()

    def _setup_state_labs(self):
        self.state_labs['pause'] = tk.Label(master=self, image=ICONS['pause'], height=36, width=36, bg="white")
        self.state_labs['play'] = tk.Label(master=self, image=ICONS['play'], height=36, width=36, bg=COLOR_GRAY2)

        self.state_labs['pause'].grid(row=1, column=0, padx=5, pady=5)
        self.state_labs['play'].grid(row=1, column=1)

        ttk.Separator(master=self, orient="vertical").grid(row=1, column=2, padx=5, pady=2, sticky="ns")

    def update_state(self, ros_msg):
        active_state = 'pause'
        if ros_msg.status_list:
            if ros_msg.status_list[-1].status == 1:
                active_state = 'play'

        if active_state == 'pause':
            self.state_labs['pause'].config(bg="white")
            self.state_labs['play'].config(bg=COLOR_GRAY2)
        else:
            self.state_labs['play'].config(bg="white")
            self.state_labs['pause'].config(bg=COLOR_GRAY2)

    def _setup_ents(self):
        for i, dim in enumerate(DIMS, start=3):
            self.pose_ents[dim] = tk.Entry(master=self, width=6, justify='right', state=tk.NORMAL, bg=COLOR_GRAY)
            self.pose_ents[dim].grid(row=1, column=i)
        ttk.Separator(master=self, orient="vertical").grid(row=1, column=6, padx=5, pady=2, sticky="ns")

    def _setup_progress(self):
        self.pro = ttk.Progressbar(master=self, orient=tk.HORIZONTAL, length=200, mode="determinate")
        self.pro.grid(row=1, column=7)

    def update_pose_ents(self, ros_msg):
        x = ros_msg.pose.pose.position.x
        y = ros_msg.pose.pose.position.y
        q = ros_msg.pose.pose.orientation
        _, _, rz_rad = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.pose_ents["x"].delete(0, tk.END)
        self.pose_ents["y"].delete(0, tk.END)
        self.pose_ents["rz"].delete(0, tk.END)
        self.pose_ents["x"].insert(tk.END, "{:4.2f}".format(x))
        self.pose_ents["y"].insert(tk.END, "{:4.2f}".format(y))
        self.pose_ents["rz"].insert(tk.END, "{:4.2f}".format(degrees(rz_rad)))

    def _get_ents(self):
        x_m = self.pose_ents['x'].get()
        y_m = self.pose_ents['y'].get()
        rz_deg = self.pose_ents['rz'].get()
        return float(x_m), float(y_m), float(rz_deg)

    def init_progress(self, x_m, y_m, rz_deg):
        self._goal_pose = (x_m, y_m, rz_deg)
        self._start_pose = self._get_ents()

    def action_feedback(self, feedback):
        xn = float(feedback.base_position.pose.position.x)
        yn = float(feedback.base_position.pose.position.y)
        xg, yg, _ = self._goal_pose
        xs, ys, _ = self._start_pose
        dist_now = sqrt((xn-xg)**2 + (yn-yg)**2)
        dist_start = sqrt((xs-xg)**2 + (ys-yg)**2)
        if dist_now < dist_start:
            self.pro["value"] = (1 - dist_now / dist_start) * 100.0
        else:
            self.pro["value"] = 0.0

    def action_done(self, status, result):
        self.pro["value"] = 0.0


class LabelFrameMoveBase(tk.LabelFrame, object):

    def __init__(self, master):
        super(LabelFrameMoveBase, self).__init__(master=master, text="move_base")

        self.frame_goto = FrameGoTo(master=self)

        self._pack_widgets()

    def _pack_widgets(self):
        self.frame_goto.pack(side='top', fill='x', padx=5, pady=5)
        # ttk.Separator(master=self, orient="horizontal").pack(side='top', fill='x')


class FrameGoTo(tk.Frame, object):

    def __init__(self, master):
        super(FrameGoTo, self).__init__(master=master)

        self.pose_ents = OrderedDict()

        self._setup_widgets()

    def _setup_widgets(self):
        tk.Label(master=self, text='Destination : ', anchor='e', width=11).pack(side='left')

        for i, dim in enumerate(DIMS, start=4):
            self.pose_ents[dim] = tk.Entry(master=self, width=6, justify='right', state=tk.NORMAL)
            self.pose_ents[dim].pack(side='left')

        ttk.Separator(master=self, orient="vertical").pack(side='left', padx=5, fill='y')

        tk.Button(master=self, text="Go To\nSimple", command=self._call_simple_goto).pack(side='left')
        tk.Button(master=self, text="Go To\nAction", command=self._call_action_goto).pack(side='left', padx=5)
        tk.Button(master=self, text="Stop", command=self._call_action_stop).pack(side='left', fill='y')

    def _call_simple_goto(self):
        try:
            x_m, y_m, rz_deg = self._get_ents()
            ROS_NODE.call_publisher(x_m=x_m, y_m=y_m, rz_deg=rz_deg)
        except Exception as e:
            print(e)
            rospy.loginfo("FrameGoTo._call_simple_goto() failed")

    def _call_action_goto(self):
        try:
            x_m, y_m, rz_deg = self._get_ents()
            self.master.master.labelframe_monitor.init_progress(x_m=x_m, y_m=y_m, rz_deg=rz_deg)
            ROS_NODE.call_action_goal(
                x_m=x_m, y_m=y_m, rz_deg=rz_deg,
                feedback_cb=self.master.master.labelframe_monitor.action_feedback,
                done_cb=self.master.master.labelframe_monitor.action_done)
        except Exception as e:
            print(e)
            rospy.loginfo("FrameGoTo._call_ation_goto() failed")

    def _call_action_stop(self):
        ROS_NODE.call_action_cancel()

    def _get_ents(self):
        x_m = self.pose_ents['x'].get()
        y_m = self.pose_ents['y'].get()
        rz_deg = self.pose_ents['rz'].get()
        return float(x_m), float(y_m), float(rz_deg)


class LabelFrameAMCL(tk.LabelFrame, object):

    """
    tk.Button call AMCL Service "/Nomotion Update" and "/Global Localization"
    """

    def __init__(self, master):
        super(LabelFrameAMCL, self).__init__(master=master, text='amcl')

        tk.Button(master=self, text='Nomotion\nUpdate', command=self._call_request_nomotion_update).pack(side='left', padx=5, pady=5)
        tk.Button(master=self, text='Global\nLocalization', command=self._call_global_localization).pack(side='left')

    def _call_request_nomotion_update(self):
        try:
            ROS_NODE.service_request_nomotion_update()
        except rospy.ServiceException as e:
            print(e)
            rospy.loginfo("LabelFrameAMC._call_service_request_nomotion_update() failed")

    def _call_global_localization(self):
        try:
            ROS_NODE.service_global_localization()
        except rospy.ServiceException as e:
            print(e)
            rospy.loginfo("LabelFrameAMC._call_service_global_localization() failed")


if __name__ == "__main__":

    try:
        ROS_NODE = RosNode()
        MainWindow()
        tk.mainloop()
    except rospy.ROSInterruptException as e:
        print(e)
