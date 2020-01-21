import rospy
import numpy as np
import math 
from autominy_msgs.msg import SteeringAngle, NormalizedSteeringCommand, SpeedCommand, Speed
from nav_msgs.msg import Odometry

#Variables
x = 0
y = 0
z = 0
steering_angle = 0.0
theta = 0.0
velocity = 0.0
l = 0.27

#Nodes and Callbacks
def callback_map(data):
    global x
    global y
    global z
    global w
    global theta
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    #eigetnlich z w
    theta = math.atan2(z,w)
    # theta = math.asin(w*y - z*x)

def callback_steering(data):
    global steering_angle
    steering_angle = data.value

def callback_velocity(data):
    global velocity
    velocity = data.value

rospy.init_node("PD_Controller", anonymous=False)
steeringPublisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
rospy.Subscriber("/actuators/steering_normalized", NormalizedSteeringCommand, callback_steering)
speedPublisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
rospy.Subscriber("/sensors/speed", Speed, callback_velocity)
rospy.Subscriber("/communication/gps/15", Odometry, callback_map)
rospy.sleep(2)

#PD Control function
# target_angle = input("Target angle: ")
target_angle = 0
error = np.abs(theta - target_angle)
print(x,y,z)
print("theta:", theta, "steering_angle:", steering_angle, "target_angle:", target_angle, "error:", error)
velocity_angle = velocity / l * np.tan(steering_angle)
i = 0
kp = 0.55
kd = 4
while  error >= 0.01:
    error = np.abs(theta - target_angle)
    u = kp * (theta - target_angle) + kd * (0.0 - velocity_angle)
    control_signal = u
    speedPublisher.publish(value=0.5)
    steeringPublisher.publish(value=control_signal)
    i += 10
    if i % 1000 == 0:
        print("u: ", u, "sig: ", control_signal, "er: ", error, "the: ", theta)
speedPublisher.publish(value=0.0)