#!/usr/bin/env python

"""
Ref:
https://github.com/ros-teleop/teleop_twist_keyboard
"""

from __future__ import print_function

import sys, select, termios, tty

import rospy
from std_msgs.msg import String

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('/keyboard', String, queue_size = 1)
    rospy.init_node('keyboard')

    try:
        rospy.loginfo("Start keyboard.py")
        while True:
            key = getKey()
            if (key == '\x03'):
                break
            output = String()
            output.data = str(key)
            pub.publish(output)

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
