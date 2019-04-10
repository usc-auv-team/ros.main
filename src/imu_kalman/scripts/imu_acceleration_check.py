from geometry_msgs.msg import Vector3Stamped
import rospy


def checkAccelZero():
	pub = rospy.Publisher('ngimu/zeroVelocity', Vector3Stamped, queue_size=10)
	accel_count =  Vector3Stamped()
	accel_count.vector.x = accel_count.vector.y = accel_count.vector.z = 0
	threshold = 25

	while True:
		if acceleration.vector.x == 0:
			accel_count.vector.x += 1
		else:
			accel_count.vector.x = 0

		if acceleration.vector.y == 0:
			accel_count.vector.y += 1
		else:
			accel_count.vector.y = 0

		if acceleration.vector.z == 0:
			accel_count.vector.z += 1
		else:
			accel_count.vector.z = 0

		if accel_count.vector.x >= threshold and accel_count.vector.y >= threshold and accel_count.vector.z >= threshold:
			message = Vector3Stamped()
			message.vector.x = True
			message.vector.y = True
			message.vector.z = True
			pub.publish(message)
			accel_count.vector.x = accel_count.vector.y = accel_count.vector.z = 0
		elif accel_count.vector.x >= threshold and accel_count.vector.y >= threshold:
			message = Vector3Stamped()
			message.vector.x = True
			message.vector.y = True
			message.vector.z = False
			pub.publish(message)
			accel_count.vector.x = accel_count.vector.y = 0
		elif accel_count.vector.x >= threshold and accel_count.vector.z >= threshold:
			message = Vector3Stamped()
			message.vector.x = True
			message.vector.y = False
			message.vector.z = True
			pub.publish(message)
			accel_count.vector.x = accel_count.vector.z = 0
		elif accel_count.vector.y >= threshold and accel_count.vector.z >= threshold:
			message = Vector3Stamped()
			message.vector.x = False
			message.vector.y = True
			message.vector.z = True
			pub.publish(message)
			accel_count.vector.y = accel_count.vector.z = 0
		elif accel_count.vector.z >= threshold:
			message = Vector3Stamped()
			message.vector.x = True
			message.vector.y = False
			message.vector.z = False
			pub.publish(message)
			accel_count.vector.x = 0
		elif accel_count.vector.y >= threshold:
			message = Vector3Stamped()
			message.vector.x = False
			message.vector.y = True
			message.vector.z = False
			pub.publish(message)
			accel_count.vector.y = 0
		elif accel_count.vector.z >= threshold:
			message = Vector3Stamped()
			message.vector.x = False
			message.vector.y = False
			message.vector.z = True
			pub.publish(message)
			accel_count.vector.z = 0


if __name__ == '__main__':
	checkAccelZero()