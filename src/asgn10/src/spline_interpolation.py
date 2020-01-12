import rospy
from autominy_msgs.msg import SteeringAngle, NormalizedSteeringCommand, SpeedCommand, Speed
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate


steering_angle = 0.0
velocity = 0.0
clicked_point_x = 0
clicked_point_y = 0
clicked_point_z = 0

def callback_clicked_point(data):
    global clicked_point_x
    global clicked_point_y
    global clicked_point_z
    clicked_point_x = data.x
    clicked_point_y = data.y
    clicked_point_z = data.z

def callback_steering(data):
    global steering_angle
    steering_angle = data.value

def callback_velocity(data):
    global velocity
    velocity = data.value

rospy.init_node("spline_interpolation", anonymous=False)
steeringPublisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
rospy.Subscriber("/actuators/steering_normalized", NormalizedSteeringCommand, callback_steering)
speedPublisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
rospy.Subscriber("/sensors/speed", Speed, callback_velocity)

SplinePublisher = rospy.Publisher("spline_interp", Marker, queue_size=10)
ClickedPointPublisher = rospy.Publisher("/clicked_point", Point , queue_size=10)
rospy.Subscriber("/clicked_point", Point, callback_clicked_point)
ClosestPointPublisher = rospy.Publisher("closest_point", Marker, queue_size=10)
rospy.sleep(1)

lane1 = np.load('lane1.npy')
lane2 = np.load('lane2.npy')
print(lane1[0:-1:2,2])

lane1_interp_x = interpolate.interp1d(lane1[0:-1:2,0], lane1[0:-1:2,1], 'cubic')
lane1_interp_y = interpolate.interp1d(lane1[0:-1:2,0], lane1[0:-1:2,2], 'cubic')

lane2_interp_x = interpolate.interp1d(lane2[0:-1:2,0], lane2[0:-1:2,1], 'cubic')
lane2_interp_y = interpolate.interp1d(lane2[0:-1:2,0], lane2[0:-1:2,2], 'cubic')

