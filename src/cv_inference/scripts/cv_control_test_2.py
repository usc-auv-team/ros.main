"""
Test script that should be able to read the angle from the ngimu/euler node
and use that angle to calculate how much to turn the sub. It should then
guide the sub towards the target more easily.
"""
#!/usr/bin/env python
from motion_controller.srv import *
from std_msgs.msg import String
import rospy
import re
import json
import time
# possible bug source
# I'm trying to read Vector3 and Vector3Stamped messages over ROS
# might have to change these a bit to make them work
# find where the geometry_msgs folder is and look for the message declaration
from geometry_msgs.msg import Vector3, Vector3Stamped

desired_yaw = 0
current_yaw = 0
frame_width = 640
frame_height = 480
frame_w_center = frame_width/2
frame_h_center = frame_height/2
default_power = 0.5
h_fov = 80 #degrees
v_fov = 64 #degrees

def motors_client(angle, power):
    # print("Before SetEnabled")
    rospy.wait_for_service('motion_controller/SetEnabled')
    # print("After SetEnabled, before Set ForwardsPower")
    rospy.wait_for_service('motion_controller/SetForwardsPower')
    # print("After SetFowardsPower, before SetYawAngle")
    rospy.wait_for_service('motion_controller/SetYawAngle')
    # print("After SetYawAngle")

    try:
        set_enabled = rospy.ServiceProxy('motion_controller/SetEnabled', SetEnabled)
        set_forwards_power = rospy.ServiceProxy('motion_controller/SetForwardsPower', SetForwardsPower)
        set_yaw_angle = rospy.ServiceProxy('motion_controller/SetYawAngle', SetYawAngle)
        m_power = set_forwards_power(power)
        m_angle = set_yaw_angle(angle)
        # m_enable = set_enabled(True)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def get_imuangle(data):
    """ This is where we try to get the angle from ngimu/euler
    The problem is that we need to test this with the sub
    This is the cpp declaration of the message published for euler

    typedef struct {
        OscTimeTag timestamp;
        float roll;
        float pitch;
        float yaw;
    } NgimuEuler;
    """
    global current_yaw
    current_yaw = data.vector.y

def calc_angle(object_center):
    """ Calculates the amount of angle needed to be added to the current
    angle. NOTE: This math might give wrong values but should give right
    or left correctly"""
    global frame_w_center, frame_h_center, frame_width, frame_height, h_fov
    pixel_displacement = object_center - frame_w_center
    pixel_fraction = pixel_displacement/(frame_width/2)
    degree_displacement = h_fov * pixel_fraction
    return degree_displacement


def callback(data):
   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   try:
        global desired_yaw, current_yaw, current_distance, frame_w_center, frame_h_center, default_power
        json_data = json.loads(data.data)
        xmin = json_data['xmin']
        print("got xmin from json: " + str(xmin))
        xmax = json_data['xmax']
        print("got xmax from json: " + str(xmax))
        ymin = json_data['ymin']
        print("got ymin from json: " + str(ymin))
        ymax = json_data['ymax']
        print("got ymax from json: " + str(ymax))

        object_center = (xmax+xmin)/2
        # calculate the change in yaw that we need
        degree_displacement = calc_angle(object_center)
        desired_yaw = desired_yaw + degree_displacement
        motors_client(desired_yaw, default_power)
   except KeyboardInterrupt:
        set_disabled()

def cv_controls_test():
    try:
        rospy.init_node('cv_controls_test',anonymous = True)
        rospy.Subscriber("cv_detection", String, callback)
        rospy.Subscriber('ngimu/euler', Vector3Stamped, get_imuangle)
        rospy.spin()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        print("Exiting")

# def zero_yaw():
#     print("Waiting for Zero Service")
#     rospy.wait_for_service('motion_controller/Zero')
#     print("After waiting for Zero service")
#
#     try:
#         zero = rospy.ServiceProxy('motion_controller/Zero', Zero)
#         call_zero = zero()
#         print(call_zero)
#         return
#
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

# def set_disabled():
#     print("Waiting for SetEnabled")
#     rospy.wait_for_service('motion_controller/SetEnabled')
#     print("After SetEnabled")
#
#     try:
#         set_enabled = rospy.ServiceProxy('motion_controller/SetEnabled', SetEnabled)
#         enable = set_enabled(False)
#         return
#
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e


if __name__ == '__main__':
    try:
        # zero_yaw()
        cv_Controls_Test()
    except (KeyboardInterrupt, SystemExit):
        print("Exiting")
        set_disabled()
