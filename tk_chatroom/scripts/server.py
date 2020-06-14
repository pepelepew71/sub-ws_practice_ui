#!/usr/bin/env python

import rospy
from tk_chatroom.srv import SendMsg, SendMsgResponse
from std_msgs.msg import String

PUBLISHER = rospy.Publisher(name='/message', data_class=String, queue_size=10)

def callback(request):
    res = "%s : %s" % (request.name, request.message)
    PUBLISHER.publish(res)
    return SendMsgResponse(res)

if __name__ == "__main__":
    rospy.init_node(name='server')
    service = rospy.Service(name='/send_msg', service_class=SendMsg, handler=callback)
    rospy.spin()
