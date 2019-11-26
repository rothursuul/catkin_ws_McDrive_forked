import rospy
import numpy as np

from autominy_msgs.msg import SteeringFeedback, SpeedCommand, NormalizedSteeringCommand, Tick
from nav_msgs.msg import Odometry


#Variables
speed = SpeedCommand()
steering = NormalizedSteeringCommand()
currentPosition_x = 0
currentPosition_y = 0
currentPosition_z = 0
current_ticks = 0
ticks_counted = [0,0,0]
#Callback Functions to get values from Subscriber nodes
def callback_position(data):
    global currentPosition_x
    global currentPosition_y
    global currentPosition_z
    currentPosition_x = data.pose.pose.position.x
    currentPosition_y = data.pose.pose.position.y
    currentPosition_z = data.pose.pose.position.z

def callback_tick(data):
    global current_ticks
    current_ticks = data.value

#Initializing Node, Publisher, Subscribers
rospy.init_node("calibration_wheeel", anonymous=False)
steeringPublisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
speedPublisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
rospy.Subscriber("/communication/gps/5", Odometry, callback_position)
rospy.Subscriber("/sensors/arduino/ticks", Tick, callback_tick)
rospy.sleep(1)

#Starting/Stopping the car
def start_car():
    speed.value = 0.2
    speedPublisher.publish(speed)

def stop_car():
    speed.value = 0.0
    speedPublisher.publish(speed)

#Calculating the stop position for going in a circle
def calculate_circle_stop (current_position_x, current_position_y, direction):
    r = 0.528484
    theta = 1.892205
    if direction == "left":
        theta =  np.pi / 2 - theta
    target_position_x = r * np.cos(theta) + current_position_x
    target_position_y = r * np.sin(theta) + current_position_y
    return target_position_x, target_position_y

#Calibration of the Wheel Sensors
print("Starting at:", "x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)

#Going straight
stop_position = currentPosition_x - 1
while currentPosition_x > stop_position:
    print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)
    ticks_counted[0] = ticks_counted[0] + current_ticks
    steering.value = 0
    steeringPublisher.publish(steering)
    speed.value = 0.2
    speedPublisher.publish(speed)
print(ticks_counted)
stop_car()

#Going left
stop_position_x, stop_position_y = calculate_circle_stop(currentPosition_x, currentPosition_y, "left")
while not ((stop_position_x - 0.3 < currentPosition_x < stop_position_x + 0.3) and (stop_position_y - 0.3 < currentPosition_y < stop_position_y + 0.3)):
    print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)
    ticks_counted[1] = ticks_counted[1] + current_ticks
    steering.value = 1.0
    steeringPublisher.publish(steering)
    speed.value = 0.2
    speedPublisher.publish(speed)
stop_car()

#Going right
stop_position_x, stop_position_y = calculate_circle_stop(currentPosition_x, currentPosition_y, "right")
while not ((stop_position_x - 0.3 < currentPosition_x < stop_position_x + 0.3) and (stop_position_y - 0.3 < currentPosition_y < stop_position_y + 0.3)):
    print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)
    ticks_counted[2] = ticks_counted[2] + current_ticks
    steering.value = -1.0
    steeringPublisher.publish(steering)
    speed.value = 0.2
    speedPublisher.publish(speed)
stop_car()

print("0:", ticks_counted[0], "1:",ticks_counted[1], "2:", ticks_counted[2])