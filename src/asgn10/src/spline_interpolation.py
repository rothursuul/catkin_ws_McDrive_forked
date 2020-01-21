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
marker3= Marker()
marker4= Marker()

def callback_clicked_point(data):
    global clicked_point_x
    global clicked_point_y
    global clicked_point_z
    clicked_point_x = data.x
    clicked_point_y = data.y
    clicked_point_z = data.z

rospy.init_node("spline_interpolation", anonymous=False)
rospy.loginfo('Publishing Splines')

SplinePublisher_1 = rospy.Publisher("spline_interp_1", Marker, queue_size=100)
SplinePublisher_2 = rospy.Publisher("spline_interp_2", Marker, queue_size=100)
SplinePublisher_3 = rospy.Publisher("spline_interp_3", Marker, queue_size=100)
SplinePublisher_4 = rospy.Publisher("spline_interp_4", Marker, queue_size=100)

rospy.sleep(0)

lane1 = np.load('lane1.npy')
lane2 = np.load('lane2.npy')

# lane1_breakpoints = [0,209,309,539,639,847,947,1176,1276]
lane1_breakpoints = [i for i in range(0,len(lane1), 100)] + [len(lane1)-1]
# lane2_breakpoints = [0,209,309,638,738,974,1047,1376,1476]
lane2_breakpoints = [i for i in range(0,len(lane2), 100)] + [len(lane2)-1]


lane1_interp_x = interpolate.interp1d(lane1[lane1_breakpoints,0], lane1[lane1_breakpoints,1], 'cubic')
lane1_interp_y = interpolate.interp1d(lane1[lane1_breakpoints,0], lane1[lane1_breakpoints,2], 'cubic')

lane2_interp_x = interpolate.interp1d(lane2[lane2_breakpoints,0], lane2[lane2_breakpoints,1], 'cubic')
lane2_interp_y = interpolate.interp1d(lane2[lane2_breakpoints,0], lane2[lane2_breakpoints,2], 'cubic')


#Get Samples
def sample_values(interpolation_x, interpolation_y, lane):
    res = []
    max = np.max(lane[:,0])
    for x in [float(j) / 100 for j in range(0, (int(max)*100), 1)]:
        res.append((float(interpolation_x(x)), float(interpolation_y(x))))
    # res.append((float(interpolation_x(max)), float(interpolation_y(max))))
    return res

sampled_lane_1 = np.array(sample_values(lane1_interp_x, lane1_interp_y, lane1))
sampled_lane_2 = np.array(sample_values(lane2_interp_x, lane2_interp_y, lane2))
plt.plot(sampled_lane_1[:,0], sampled_lane_1[:,1])
plt.show()

#Publish the markers
def callback(event):

    def publish_points(marker,sampled_lane, id, cond):
        marker.type = marker.LINE_STRIP
        marker.header.frame_id= id
        marker.action = marker.ADD
        marker.points = []

        # marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # marker color
        if cond:
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        # marker orientation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        for x in range(len(sampled_lane)):
            c = Point()
            if cond:
                c.x = sampled_lane[x,0]
                c.y = sampled_lane[x,1]
                c.z = 0.0
            else:
                c.x = sampled_lane[x,1]
                c.y = sampled_lane[x,2]
                c.z = 0.0

            marker.points.append(c)
        return marker

    SplinePublisher_1.publish(publish_points(marker1, sampled_lane_1, "map", True))
    SplinePublisher_2.publish(publish_points(marker2, sampled_lane_2, "map", True))
    SplinePublisher_3.publish(publish_points(marker3, lane1, "map", False))
    SplinePublisher_4.publish(publish_points(marker4, lane2, "map", False))
    rospy.Rate(100).sleep()


rospy.Timer(rospy.Duration(0.0001),callback, False)
rospy.spin()