#!/usr/bin/env python

from amn_msgs.srv import *
from studs_defines import *
import std_msgs.msg
import rospy

def init_module():
    rospy.init_node("stduds_test")
    rospy.wait_for_service(SET_MODE_SRV)
    try:
        mode_srv = rospy.ServiceProxy(SET_MODE_SRV, SetMode)
        response = mode_srv(std_msgs.msg.Header(), 1)
        return response.mode
    except rospy.ServiceException, e:
        print "Set mode service call failed: %s"%e

if __name__ == "__main__":
    init_module()
