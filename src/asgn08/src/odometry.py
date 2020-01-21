import rospy
import numpy as np
import math
from autominy_msgs.msg import Speed, SteeringAngle
from nav_msgs.msg import Odometry

velocity = 0
phi = 0
l = 0.27
qx = 0
qy = 0
theta = 0
res = Odometry()
res.header.frame_id = "map"
res.child_frame_id = "base_link"
text_output = ""


#Callback Functions for Subscriber Nodes
def callback_velocity(data):
    global velocity
    velocity = data.value

def callback_phi(data):
    global phi
    phi = data.value

def callback_map(data):
    global qx
    global qy
    global theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    theta = data.pose.pose.orientation.w

#Initializing Node and Subscribers/Publishers
rospy.init_node("Odometry", anonymous=False)
rospy.Subscriber("/sensors/speed", Speed, callback_velocity)
rospy.Subscriber("/sensors/steering", SteeringAngle, callback_phi)
rospy.Subscriber("/sensors/localization/filtered_map", Odometry, callback_map)
odometryPublisher = rospy.Publisher("odometry_calc", Odometry, queue_size = 10)
last_time = rospy.Time.now().to_sec()
rospy.sleep(0)

#Global Variables
#print(x,y,theta)

#Odometry Calculatio
def odometry_callback(event):
    global qx
    global qy
    global t0
    global res
    global velocity
    global theta
    global phi
    global text_output
    global last_time


    #print(get_velocities())
    delta_t = rospy.Time.now().to_sec() - last_time
    last_time = rospy.Time.now().to_sec()

    #get velocities
    velocity_x = velocity * np.cos(theta)
    velocity_y = velocity * np.sin(theta)
    velocity_theta = velocity / l * math.tan(phi)
    print(x,y)
    print(delta_t)
    res.pose.pose.position.x = x + delta_t * velocity_x
    res.pose.pose.position.y = y + delta_t * velocity_y
    res.pose.pose.orientation.w = np.cos((theta + delta_t * velocity_theta) / 2)
    res.pose.pose.orientation.z = np.sin((theta + delta_t * velocity_theta) / 2)
    odometryPublisher.publish(res)
    rospy.Rate(100).sleep()

rospy.Timer(rospy.Duration(0.0001), odometry_callback, False)
rospy.spin()