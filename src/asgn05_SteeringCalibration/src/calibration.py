import rospy

from autominy_msgs.msg import SteeringFeedback, SpeedCommand, NormalizedSteeringCommand
from nav_msgs.msg import Odometry

rospy.init_node("calibration", anonymous=False)
#---Subscribe to the Steering_Angle Topic to get the current value for calculation---
steeringAngle = 0
def callback_steering(data):
    global steeringAngle
    steeringAngle = data.value

def get_current_steering_angle():
    rospy.Subscriber("/sensors/arduino/steering_angle", SteeringFeedback, callback_steering)
    rospy.sleep(1)

get_current_steering_angle()
print("Test: The Current_Steering_Angle is: ", steeringAngle)
#------------------------------------------------------------------------------------

#---Subscribe to the current position------------------------------------------------
currentPosition_x = 0
currentPosition_y = 0
currentPosition_z = 0

def callback_position(data):
    global currentPosition_x
    global currentPosition_y
    global currentPosition_z
    currentPosition_x = data.pose.pose.position.x
    currentPosition_y = data.pose.pose.position.y
    currentPosition_z = data.pose.pose.position.z


def get_current_position():
    rospy.Subscriber("/communication/gps/11", Odometry, callback_position)
    rospy.sleep(1)
get_current_position()
print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)
#------------------------------------------------------------------------------------

#---Starting the calibration---------------------------------------------------------
#Values and Publishers
speed = SpeedCommand()
speed.value = 0.2
steering = NormalizedSteeringCommand()
steering.value = 1

steeringPublisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
speedPublisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
rospy.sleep(1)

#1)Steering angle calibration
#get_current_position()
steeringPublisher.publish(steering)
speedPublisher.publish(speed)
rospy.sleep(3)
print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)
rospy.sleep(3)
print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)
speed.value = 0
speedPublisher.publish(speed)

#2)calibration
#2-1)leftward direction
steering.value = 1.0
steeringPublisher.publish(steering)
while currentPosition_y != 2.2:
    speed.value = 0.1
    speedPublisher.publish(speed)
speed.value = 0.0
speedPublisher.publish(speed)
print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)

#2-2) straight
target_pos_1 = currentPosition_x + 2
while currentPosition_x != target_pos_1:
    speed.value = 0.1
    speedPublisher.publish(speed)
speed.value = 0.0
speedPublisher.publish(speed)

#2-3)leftward direction
steering.value = -1.0
steeringPublisher.publish(steering)
while currentPosition_y != 2.2:
    speed.value = 0.1
    speedPublisher.publish(speed)
speed.value = 0.0
speedPublisher.publish(speed)
print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)
