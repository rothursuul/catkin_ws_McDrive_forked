import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
#IMPORT MAP FROM STEFAN
from steering_pid import SteeringPID
from map import Lane, Map
import tf.transformations

odometry = Odometry()
x, y, z, qx, qy, qz, qw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

def callback_map(data):
    global odometry
    global x
    global y
    global z
    global qx
    global qy
    global qz
    global qw
    odometry = data
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w

rospy.init_node("auto", anonymous=False)
rospy.Subscriber("/sensors/localization/filtered_map", Odometry, callback_map)
rospy.sleep(1)

#Caclulate desired yaw angle
map = Map()
controller = SteeringPID(odometry)
current_position = np.array([x,y])
current_orientation = [qx,qy,qz,qw]
current_lane = 0

lookahead_point = map.lanes[current_lane].lookahead_point(current_position, 0.5)
_, _, yaw = tf.transformations.euler_from_quaternion(current_orientation)
diff = (lookahead_point[0] - current_position)
delta = math.atan2(diff[0,1], diff[0,0])
controller.desired_angle = np.pi