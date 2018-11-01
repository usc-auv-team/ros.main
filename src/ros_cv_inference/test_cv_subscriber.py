#!/usr/bin/env python
import re
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    datastring = re.sub('[\{]',"",data.data)
    xmin_raw,ymin_raw,xmax_raw,ymax_raw,category_raw = datastring.split(',');
    xmin = re.sub("\D","",xmin_raw)
    ymin = re.sub("\D","",ymin_raw)
    xmax = re.sub("\D","",xmax_raw)
    ymax = re.sub("\D","",ymax_raw)
    category = re.sub("/^[A-Za-z]+$/","",category_raw)
    print(xmin, ymin, xmax, ymax, category)
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("cv_detection", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
