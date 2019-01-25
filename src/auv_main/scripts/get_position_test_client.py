#!/usr/bin/env python

import sys
import rospy
from auv_main.srv import *

def test():
    rospy.wait_for_service('get_position')

    try:
        pos_srv_prox = rospy.ServiceProxy('get_position', GetPosition)
        resp1 = pos_srv_prox()
        print("Position= " + str(resp1.x) + ", " + str(resp1.y) + ", " + str(resp1.z))
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    test();