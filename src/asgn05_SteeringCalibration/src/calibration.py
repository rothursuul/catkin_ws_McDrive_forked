import rospy

from autominy_msgs.msg import SteeringFeedback, SpeedCommand, NormalizedSteeringCommand
from nav_msgs.msg import Odometry

#set the current car
currentCar = 7

#---Subscribe to the Steering_Angle Topic to get the current value for calculation---
steeringAngle = 0
def get_info():
def callback_steering(data):
    global steeringAngle
    steeringAngle = data.value

def get_current_steering_angle():
    rospy.init_node("steeringAngle", anonymous=False)
    rospy.Subscriber("/sensors/arduino/steering_angle", SteeringFeedback, callback_steering)
    rospy.spin()
print("here1")

get_current_steering_angle()
print("Test: The Current_Steering_Angle is: %s", steeringAngle)
#------------------------------------------------------------------------------------
print("here2")

#---Subscribe to the current position---
currentPosition = 0
def callback_position(data):
    global currentPosition
    currentPosition = data.value

def get_current_position():
    rospy.init_node("currentPosition", anonymous=False)
    rospy.Subscriber("/communication/gps/%s",currentCar, Odometry, callback_position)
    rospy.spin()
get_current_position()
print("Test: The Current_Steering_Position is: %s", currentPosition)
#----------------------------------------

#---Starting the calibration---
#Values and Publishers
speed = SpeedCommand()
speed.value = 0
steering = NormalizedSteeringCommand()
steering.value = 0

rospy.init_node("driveNode")
steeringPublisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
speedPublisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)

#1)Driving in a circle leftward direction

#noting the current position
get_current_position()
starting_pos_1 = currentPosition
#driving
steeringPublisher.publish(0.1)
speedPublisher.publish(0.1)
print("here!")
#keep driving until back at the start
#while starting_pos_1 != currenPosition:
 #   speedPublisher.publish(0.1)
  #  get_current_position()