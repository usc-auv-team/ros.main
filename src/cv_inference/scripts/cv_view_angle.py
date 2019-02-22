"""
Takes in computer vision data and returns an angle to the object
"""

#---IN PROGRESS, NONE OF THE CODE HERE IS CORRECT ---

#!/usr/bin/env python
from motion_controller.srv import *
from std_msgs.msg import String
from msg import CV_Msg
import rospy
import re
import json
import time

current_angle = 0
current_distance = 0
frame_w_center = 640/2
frame_h_center = 480/2
default_power = 0.5

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global current_angle, current_distance, frame_w_center, frame_h_center, default_power
    json_data = json.loads(data.data)
    distance = json_data['distance']
    print("distance: " + str(distance))
    xmax = json_data['xmax']
    print("got xmax from json: " + str(xmax))
    xmin = json_data['xmin']
    print("got xmin from json: " + str(xmin))
    object_center = (xmax+xmin)/2
    if(distance > 0):
        if(object_center > frame_w_center):
            current_angle+=1
            print("moving right")
        if(object_center < frame_w_center):
    	    current_angle-=1
            print("moving left")
    # instead of controlling motors, just send the angle and distance
    # change current_angle by the amount given by ros
    # figure out how to get angle from ROS
    """
    TODO:
    Get ROS angle
    increment this angle
    publish angle and distance
    """
    """  Sample Code
    sent_str = "{" + "\"angle\":" + str(angle) + ", \"distance\":" + str(distance) + "}"
    pub.publish(sent_str)
    """



def cv_localization():
    rospy.init_node('cv_localization',anonymous = True)
    rospy.Subscriber("cv_detection", String, callback)
    # subscribe to angle data here

    #--- Start Publisher Setup ---
    refresh_rate = 50
    pub = rospy.Publisher('cv_localization', String, queue_size=10)
    rate = rospy.Rate(refresh_rate) #10hz rate, change if needed
    seq_id = 41
    #--- End Publisher Setup ---

    rospy.spin()

if __name__ == '__main__':
    cv_localization()
