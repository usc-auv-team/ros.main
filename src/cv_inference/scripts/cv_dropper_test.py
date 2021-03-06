#!/usr/bin/env python

# from motion_controller.srv import *
from std_msgs.msg import String
import rospy
import math
import re
import json
import time

class ObjectCenter:
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y

    def set_center(self, x_min, x_max, y_min, y_max):
        self.x = x_min + (x_max - x_min) / 2
        self.y = y_min + (y_max - y_min) / 2

    def get_dot_product(self, vector1, vector2):
        return vector1[0]*vector2[0] + vector1[1]*vector2[1]

    def get_magnitude(self, vector):
        return math.sqrt(vector[0]**2 + vector[1]**2)

    # Will return the angle between itself and 0 (forwards)
    def get_angle(self, img_width, img_height):
        # Base point is (img_width/2, img_height) and vectors are calculated with that 
        base_vector = (0, 0 - img_height) # 0 for first because the calculation will always evaluate to that
        object_vector = (self.x - (img_width / 2), self.y - img_height)
        angle = math.degrees(math.acos(self.get_dot_product(base_vector, object_vector) /
            (self.get_magnitude(base_vector) * self.get_magnitude(object_vector))))
        return 0 - angle if self.x > (img_width/2) else angle


current_angle = 0
default_power = 0.5
frame_width = 640
frame_height = 480
current_box_location = ObjectCenter()
angle_threshhold = 8 # Threshhold for angle between AUV and box
distance_threshhold = 10 # Threshhold for distance between the AUV and box 

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

def cv_motors_client(angle, power):
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


def step(direction):
    global current_angle
    power = 10
    motor_duration = 0.25

    if direction == "forward":
        cv.cv_motors_client(current_angle, power)
        time.sleep(motor_duration)
        cv.cv_motors_client(current_angle, 0)
    elif direction == "backward":
        cv.cv_motors_client(current_angle, 0 - power)
        time.sleep(motor_duration)
        cv.cv_motors_client(current_angle, 0)
    elif direction == "left":
        cv.cv_motors_client(current_angle + 10, power)
        current_angle += 10
        time.sleep(motor_duration)
        cv.cv_motors_client(current_angle, 0)
    elif direction == "right":
        cv.cv_motors_client(current_angle - 10 ,power)
        current_angle -= 10
        time.sleep(motor_duration)
        cv.cv_motors_client(current_angle, 0)

def drop_marker():
    print("Waiting for marker_dropper service")
    rospy.wait_for_service("marker_dropper/drop_marker")
    print("Attempting to drop marker!")
    try:
        set_enabled = rospy.ServiceProxy('marker_dropper/drop_marker', DropMarker)
        set_enabled()
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

def callback(data):
   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   try:
        global current_angle, default_power, current_box_location, frame_height, \
            frame_width, angle_threshhold, distance_threshhold
        json_data = json.loads(data.data)
        x_min = json_data['xmin']
        print("got xmin from json: " + str(x_min))
        x_max = json_data['xmax']
        print("got xmax from json: " + str(x_max))
        y_min = json_data['ymin']
        print("got ymin from json: " + str(y_min))
        y_max = json_data['ymax']
        print("got ymax from json: " + str(y_max))
        
        # Set object center
        current_box_location.set_center(x_min, x_max, y_min, y_max)

        # Get angle of box center with respect to the y-axis
        angle = current_box_location.get_angle(frame_width, frame_height)
        print("Current angle difference is ", angle)

        # If aligned properly move so that the box is under the AUV
        if abs(angle) <= angle_threshhold:
            # Move accordingly
            print("Angle within threshold so I want to move forwards or backwards")

            # Get distance between AUV and box
            distance = (frame_height/2) - current_box_location.y
            print("Distance from target is ", distance)
            if abs(distance) <= distance_threshhold:
                print("AUV is in position!")
                # drop_marker()

            elif distance > 0: # Move forward
                print("I want to move forward!")
                step("forward")

            elif distance < 0: # move backwards
                print("I want to move backward!")
                step("backward")

        # Align self with it if needed
        else:
            print("Correcting my angle")
            if angle < 0: # Turn right
                print("I want to turn right")
                step("right")

            else: # Turn left
                print("I want to turn left")
                step("left")

        # Add sleep to not jam pid
        # This is a really basic fix; snazzy one coming later
        time.sleep(5)
        # Now Repeat

   except KeyboardInterrupt:
        set_disabled()

def cv_dropper_test():
    try:
        rospy.init_node('cv_dropper_test',anonymous = True) # Init node
        rospy.Subscriber("cv_detection", String, callback) # Subscribe to cv_detection and perform callback function
        rospy.spin() # Loop to keep going
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        print("Exiting")

def local_test():
    obj = ObjectCenter(240,50)
    angle = obj.get_angle(640,480)
    print(angle)


if __name__ == '__main__':
    try:
        print("Started!")
        # local_test()
        cv_dropper_test()
    except (KeyboardInterrupt, SystemExit):
        print("Exiting")
        set_disabled()
