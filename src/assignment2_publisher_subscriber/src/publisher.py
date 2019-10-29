import rospy

from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand

def driveCommand():
	pub = rospy.Publisher('actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
	rospy.init_node('driveCommand')
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		steeringCommand = 1.0
		rospy.loginfo(steeringCommand)
		pub.publish(steeringCommand)
		rate.sleep

if __name__ == '__main__':
	try:
		driveCommand()
	except rospy.ROSInterruptException:
		pass