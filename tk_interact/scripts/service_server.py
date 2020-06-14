#!/usr/bin/env python

import rospy
from tk_interact.srv import Calculate, CalculateResponse
from std_msgs.msg import String

def callback(request):
    try:
        res = eval(str(request.expression))
    except:
        res = "not an expression!"
    return CalculateResponse(str(res))

if __name__ == "__main__":

    rospy.init_node(name='service_server')
    service = rospy.Service(name='/calculate', service_class=Calculate, handler=callback)
    rospy.spin()