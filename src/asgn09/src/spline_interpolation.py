import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from nav_msgs.msg import Odometry


clicked_point_x = 0
clicked_point_y = 0
clicked_point_z = 0
marker1 = Marker()
marker2= Marker()

def callback_clicked_point(data):
    global clicked_point_x
    global clicked_point_y
    global clicked_point_z
    clicked_point_x = data.x
    clicked_point_y = data.y
    clicked_point_z = data.z

rospy.init_node("spline_interpolation", anonymous=False)

SplinePublisher_1 = rospy.Publisher("spline_interp_1", Marker, queue_size=100)
SplinePublisher_2 = rospy.Publisher("spline_interp_2", Marker, queue_size=100)
ClickedPointPublisher = rospy.Publisher("/clicked_point", Point , queue_size=10)
rospy.Subscriber("/clicked_point", Point, callback_clicked_point)
ClosestPointPublisher = rospy.Publisher("closest_point", Marker, queue_size=10)
MarkerPublisher = rospy.Publisher("closest_point", Marker, queue_size=10)
odometryPublisher = rospy.Publisher("odometry_calc", Odometry, queue_size = 10)

rospy.sleep(0)

lane1 = np.load('lane1.npy')
lane2 = np.load('lane2.npy')

lane1_breakpoints = [0,209,309,539,639,847,947,1176,1276]
lane2_breakpoints = [0,209,309,638,738,974,1047,1376,1476]

lane1_interp_x = interpolate.interp1d(lane1[lane1_breakpoints,0], lane1[lane1_breakpoints,1], 'cubic')
lane1_interp_y = interpolate.interp1d(lane1[lane1_breakpoints,0], lane1[lane1_breakpoints,2], 'cubic')

lane2_interp_x = interpolate.interp1d(lane2[lane2_breakpoints,0], lane2[lane2_breakpoints,1], 'cubic')
lane2_interp_y = interpolate.interp1d(lane2[lane2_breakpoints,0], lane2[lane2_breakpoints,2], 'cubic')


#Get Samples
def sample_values(interpolation_x, interpolation_y, lane):
    res = []
    max = np.max(lane[:,0])
    for x in range(0, int(max)+1):
        res.append((float(interpolation_x(x)), float(interpolation_y(x))))
    res.append((float(interpolation_x(max)), float(interpolation_y(max))))
    return res

sampled_lane_1 = np.array(sample_values(lane1_interp_x, lane1_interp_y, lane1))
sampled_lane_2 = np.array(sample_values(lane2_interp_x, lane2_interp_y, lane2))
plt.plot(sampled_lane_1[:,0], sampled_lane_1[:,1])
# plt.show()

#Publish the markers
def publish_points(marker,sampled_lane, id):
    marker.type = marker.LINE_STRIP
    marker.header.frame_id= id
    marker.action = marker.ADD
    marker.points = []
    for x in range(len(sampled_lane)):
        c = Point()
        c.x = sampled_lane[x,0]
        c.y = sampled_lane[x,1]
        c.z = 0.0
        marker.points.append(c)
    return marker

SplinePublisher_1.publish(publish_points(marker1, sampled_lane_1, "id_1"))
SplinePublisher_2.publish(publish_points(marker2, sampled_lane_2, "id_2"))
rospy.spin()

