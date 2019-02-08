#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Vector3Stamped

def acc():
    pub = rospy.Publisher('earthAccel', Vector3Stamped, queue_size=10)
    rospy.init_node('acc', anonymous=False)
    rate = rospy.Rate(5) # 10hz

    while not rospy.is_shutdown():
        send = Vector3Stamped();
        send.header.stamp = rospy.Time.now()
        send.vector.x = 1
        send.vector.y = .001
        send.vector.z = .0001

        pub.publish(send)

        rate.sleep()

if __name__ == '__main__':
    try:
        acc()
    except rospy.ROSInterruptException:
        pass