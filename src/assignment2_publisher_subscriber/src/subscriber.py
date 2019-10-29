import rospy
from autominy_msgs.msg import Speed

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Current Speed is: %", data.data)

def speedData():
	rospy.init_node('speedData', anonymous=True)
	rospy.Subscriber("/sensors/speed", Speed, callback)
	rospy.spin
