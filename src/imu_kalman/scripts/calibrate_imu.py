from geometry_msgs.msg import Vector3Stamped
import rospy

count = 0
acceleration_bias = Vector3Stamped()
accel_zero_count = 0

def addAccelSample(acceleration):
	global count
	global acceleration_bias
	acceleration_bias.vector.x += acceleration.vector.x
	acceleration_bias.vector.y += acceleration.vector.y
	acceleration_bias.vector.z += acceleration.vector.z
	count += 1

def calibrate():
	global count
	global acceleration_bias
	acceleration_bias.vector.x = 0.0
	acceleration_bias.vector.y = 0.0
	acceleration_bias.vector.z = 0.0
	subscription = rospy.Subscriber('ngimu/earthAccel', Vector3Stamped, addAccelSample)
	
	while True:
		if count >= 1024:
			subscription.unregister()
			break
			
	acceleration_bias.vector.x /= count
	acceleration_bias.vector.y /= count
	acceleration_bias.vector.z /= count

	return acceleration_bias

def checkAccelZero(acceleration):
	if acceleration.vector.x == 0 and acceleration.vector.y == 0 and acceleration.vector.z == 0:
		accel_zero_count += 1
	else:
		accel_zero_count = 0

def movement_end_check(acceleration_bias):
	pub = rospy.Publisher('ngimu/zeroVelocity', bool, queue_size=10)
	rospy.Subscriber('ngimu/earthAccel', Vector3Stamped, checkAccelZero)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		pub.publish(True)
		rate.sleep()
