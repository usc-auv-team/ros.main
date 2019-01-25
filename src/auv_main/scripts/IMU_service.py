#!/usr/bin/env python
 
## AUTHORS
## Physics by Yash Chandak
## ROS Interface by Matthew Fala

## INFO
## This is the IMU Service for collecting accelleration information and
## providing location coordinates in meters

## IMU Service USAGE
#
# 1) Wait for the service to load 
#          rospy.wait_for_service('get_position')
#
# 2) Create a Service Proxy
#        pos_proxy = rospy.ServiceProxy('get_position', GetPosition)
#        current_position = pos_proxy()
#
# 3) Get coordinates
#       current_x = current_position.x
#       current_y = current_position.y
#       current_z = current_position.z
#
# 4) Make sure to handle exceptions
#       except rospy.ServiceException, e:
#            print "Service call failed: %s"%e

## CONVENTION: D is the z axis and is used as a standard up, not down!!!

from auv_main.srv import *
from geometry_msgs.msg import Vector3Stamped
import rospy
import sys

first_update = True

past_an = 0.0
past_ae = 0.0
past_ad = 0.0
past_vn = 0.0
past_ve = 0.0
past_vd = 0.0
past_dn = 0.0
past_de = 0.0
past_dd = 0.0

past_time_stamp = rospy.Time(0)

def update_pos(data):

    global past_time_stamp
    global first_update

    # only unpack on first_update
    if first_update:

        past_time_stamp = data.header.stamp
        first_update = False
        return
    
    # is this really nessesary?
    global past_an
    global past_ae
    global past_ad
    global past_vn
    global past_ve
    global past_vd
    global past_dn
    global past_de
    global past_dd
    
    # unpack data
    an = data.vector.x
    ae = data.vector.y
    ad = data.vector.z
    delta_t = (data.header.stamp - past_time_stamp).to_sec()

    # Super basic filter to remove IMU output that is in high magnitude (more in Notes below)
    if (abs(an) > 1.0*10**3):
        an = past_an
    if (abs(ae) > 1.0*10**3):
        ae = past_ae
    if (abs(ad) > 1.0*10**3):
        ad = past_ad

    # Another way to calculate distance using acceleration
    # Both methods should theoretically come to the same value, but Danny claimed my formula was off so I implemented his
    vn = past_vn + an*delta_t
    ve = past_ve + ae*delta_t
    vd = past_vd + ad*delta_t
    dn = past_dn + vn*delta_t + 0.5*an*delta_t**2
    de = past_de + ve*delta_t + 0.5*ae*delta_t**2
    dd = past_dd + vd*delta_t + 0.5*ad*delta_t**2

    # Put current values in the initial variables
    past_an = an
    past_ae = ae
    past_ad = ad
    past_vn = vn
    past_ve = ve
    past_vd = vd
    past_dn = dn
    past_de = de
    past_dd = dd
    past_time_stamp =  data.header.stamp

def get_position(empty):
    return GetPositionResponse(past_dn, past_de, past_dd)
    

"""
Notes:
There is a calibration error that is causing there to be a slight acceleration even when the IMU is sitting still.
We need to find a way to calibrate the IMU before testing.

The IMU outputs seemingly random values that exceed 1e10 in magnitude for acceleration. We need to use a simple
filter (use the previous value) to remove this. I have done this already.

In EarthAccel, the IMU starts to move after 5000 lines. In EarthAccelShortened, I removed those lines. I believe this
file is only when the IMU is not moving. Need to collect better data.

Need to implement the Kalman Filter.

Need to collect data of the IMU moving in a circle.

Need to figure out the units of the data given.
"""

def get_position_server():

    # init node
    rospy.init_node('get_position_server')

    # subscribe earthAccel
    rospy.Subscriber("earthAccel", Vector3Stamped, update_pos)
    
    # start service    
    s = rospy.Service('get_position', GetPosition, get_position)
    print "IMU service ready."

    rospy.spin()

if __name__ == "__main__":
    get_position_server()
