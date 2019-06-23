import rospy
import time
from std_msgs import String
from geometry_msgs.msg import Vector3, Vector3Stamped


class MotorController(object):
	"""
	Intantiates a controller for the moters to avoid interfacing with them directly.
	This way the PID loop won't keep jamming.
	"""
	def __init__(self):
		super(MotorController, self).__init__()
		self.yaw = 0
		self.yaw_storage = []
		self.time = time.time()

	def is_yaw_moving(self):
		rospy.init_node('motor_listener', anonymous=True)
		rospy.Subscriber("ngimu/euler", Vector3Stamped, is_moving_callback)

		rospy.spin()

	def is_yaw_moving_callback(self, data):
		# Get current yaw
		# curr_yaw = 4 # This is the new yaw from data. 4 is a placeholder

		# Check it with the old yaw


		# Get speed from the change in the angle
		# May need a frame counter

		# Store the yaw in a list and only operate every 20 frames
		# If the variance is close to 0, we are stable
		# If the variance is high, our yaw is changing

		# Based on calculations, print whether moving or still
		# --
		yaw_storage.append(data)
		if time.time() - self.time >= 1:
			print(len(self.yaw_storage))
			self.yaw_storage.clear()

		print(data)


def main():
	print("Main started!")
	motor_controller = MotorController()
	motor_controller.is_yaw_moving()

if __name__ == '__main__':
	main()