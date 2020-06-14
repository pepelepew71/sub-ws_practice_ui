#!/usr/bin/env python

from __future__ import print_function

import sys

if sys.version_info >= (3, 0):
    import tkinter as tk
    from tkinter import scrolledtext as ScrolledText
else:
    import Tkinter as tk
    import ScrolledText

import rospy
from tk_chatroom.srv import SendMsg
from std_msgs.msg import String


class RosClient():

    def __init__(self, main_frame):
        self.main_frame = main_frame
        rospy.init_node(name='client', anonymous=True)
        self.main_frame.username = rospy.get_name().lstrip("/")
        self._subscribe()

    def _subscribe(self):
        rospy.Subscriber(name="/message", data_class=String, callback=self._sub_callback)
        rospy.sleep(0.5)

    def _sub_callback(self, response):
        self.main_frame.board.config(state='normal')
        self.main_frame.board.insert(tk.END, response.data + "\n")
        self.main_frame.board.config(state='disabled')
        self.main_frame.board.see(tk.END)

    def send_msg_by_service(self, text):
        if not rospy.is_shutdown():
            try:
                rospy.wait_for_service(service='/send_msg')
                service = rospy.ServiceProxy(name='/send_msg', service_class=SendMsg)
                service(self.main_frame.username, text)
            except rospy.ServiceException:
                print("RosClient.send_msg_by_service() failed")
                sys.exit(1)


class MainFrame(tk.Frame):

    def __init__(self, root=None):
        tk.Frame.__init__(self, master=root)
        self.pack(side='top', fill='both', padx=5, pady=2, expand=False)
        self.username = "Anonymous"
        self.ros_client = RosClient(main_frame=self)
        self._setup_board()
        self._setup_input()

    def _setup_board(self):
        lab = tk.Label(self, text='Board', anchor='w')
        lab.pack(side='top', pady=2, fill='both', expand=False)
        self.board = ScrolledText.ScrolledText(self, height=15, state='disabled', wrap=tk.WORD)
        self.board.pack(side='top', fill='both', expand=True)

    def _setup_input(self):
        lab = tk.Label(self, text=self.username + ' Say :', anchor='w')
        lab.pack(side='top', pady=2, fill='both', expand=False)
        self.btn = tk.Button(self, text='Send', command=self._send_input)
        self.btn.pack(side='right', fill='both', expand=False)
        self.input = ScrolledText.ScrolledText(self, height=4, wrap=tk.WORD)
        self.input.pack(side='left', fill='both', padx=(0, 5), expand=True)
        self.input.focus()
        self.input.insert("insert", "\n")
        self.input.bind("<KP_Enter>", self._send_input)
        self.input.bind("<Return>", self._send_input)
        self.input.bind("<Shift-Return>", self._dummy) # disable <Return> event

    def _send_input(self, event=None):
        text = self.input.get("1.0", tk.END)
        text = u''.join(text).encode('utf-8')
        text = text.strip("\n")
        self.input.delete("1.0", tk.END)
        if text:
            self.ros_client.send_msg_by_service(text)
        else:
            pass

    def _dummy(self, event=None):
        pass


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("400x400")
    root.title("ROS Chat Room")
    MainFrame(root=root)
    root.mainloop()

