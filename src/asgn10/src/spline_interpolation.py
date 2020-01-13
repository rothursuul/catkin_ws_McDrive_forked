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

lane1_breakpoints = [0,209,309,539,639,847,947,1176,1276]
lane2_breakpoints = [0,209,309,638,738,974,1047,1376,1476]

lane1_interp_x = interpolate.interp1d(lane1[lane1_breakpoints,0], lane1[lane1_breakpoints,1], 'cubic')
lane1_interp_y = interpolate.interp1d(lane1[lane1_breakpoints,0], lane1[lane1_breakpoints,2], 'cubic')

lane2_interp_x = interpolate.interp1d(lane2[lane2_breakpoints,0], lane2[lane2_breakpoints,1], 'cubic')
lane2_interp_y = interpolate.interp1d(lane2[lane2_breakpoints,0], lane2[lane2_breakpoints,2], 'cubic')

x_plot = np.arange(0,13)
y_plot = lane1_interp_y(x_plot)
# plt.plot(x_plot, y_plot)
#plt.show()

def sample_values(interpolation_x, interpolation_y, lane):
    res = []
    max = np.max(lane[:,0])
    for x in range(0, int(max)):
        res.append((float(interpolation_x(x)), float(interpolation_y(x))))
    res.append((float(interpolation_x(max)), float(interpolation_y(max))))
    return res

sampled_lane_1 = np.array(sample_values(lane1_interp_x, lane1_interp_y, lane1))
sampled_lane_2 = np.array(sample_values(lane2_interp_x, lane2_interp_y, lane2))
print(lane1[lane1_breakpoints,0].shape)
plt.plot(sampled_lane_1[:,0], sampled_lane_1[:,1])
plt.show()