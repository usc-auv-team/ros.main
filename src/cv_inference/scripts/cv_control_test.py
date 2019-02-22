#!/usr/bin/env python

from motion_controller.srv import *
from std_msgs.msg import String
import rospy
import re
import json
import time

current_angle = 0
current_distance = 0
frame_w_center = 640/2
frame_h_center = 480/2
default_power = 0.5

def cv_Motors_Client(angle, power):
    print("Before SetEnabled")
    rospy.wait_for_service('motion_controller/SetEnabled')
    print("After SetEnabled, before Set ForwardsPower")
    rospy.wait_for_service('motion_controller/SetForwardsPower')
    print("After SetFowardsPower, before SetYawAngle")
    rospy.wait_for_service('motion_controller/SetYawAngle')
    print("After SetYawAngle")

    try:
        set_enabled = rospy.ServiceProxy('motion_controller/SetEnabled', SetEnabled)
        set_forwards_power = rospy.ServiceProxy('motion_controller/SetForwardsPower', SetForwardsPower)
        set_yaw_angle = rospy.ServiceProxy('motion_controller/SetYawAngle', SetYawAngle)
        m_power = set_forwards_power(power)
        m_angle = set_yaw_angle(angle)
        m_enable = set_enabled(True)
        if(power == 0):
            enable = set_enabled(False)
        return

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def callback(data):
   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   try:
        global current_angle, current_distance, frame_w_center, frame_h_center, default_power
        json_data = json.loads(data.data)
        xmax = json_data['xmax']
        print("got xmax from json: " + str(xmax))
        xmin = json_data['xmin']
        print("got xmin from json: " + str(xmin))
        object_center = (xmax+xmin)/2
        if(object_center > frame_w_center):
            current_angle+=1
            print("moving right")
        if(object_center < frame_w_center):
    	     current_angle-=1
             print("moving left")
        cv_Motors_Client(current_angle, default_power)
   except KeyboardInterrupt:
        set_disabled()

def cv_Controls_Test():
    try:
        rospy.init_node('cv_Controls_Test',anonymous = True)
        rospy.Subscriber("cv_detection", String, callback)
        rospy.spin()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print("Exiting")

def zero_yaw():
    print("Waiting for Zero Service")
    rospy.wait_for_service('motion_controller/Zero')
    print("After waiting for Zero service")

    try:
        zero = rospy.ServiceProxy('motion_controller/Zero', Zero)
        call_zero = zero()
        print(call_zero)
        return

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_disabled():
    print("Waiting for SetEnabled")
    rospy.wait_for_service('motion_controller/SetEnabled')
    print("After SetEnabled")

    try:
        set_enabled = rospy.ServiceProxy('motion_controller/SetEnabled', SetEnabled)
        enable = set_enabled(False)
        return

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    try:
        zero_yaw()
        cv_Controls_Test()
    except (KeyboardInterrupt, SystemExit):
        print("Exiting")
        set_disabled()
