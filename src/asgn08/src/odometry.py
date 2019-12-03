import rospy
import numpy as np
from autominy_msgs.msg import Speed, SteeringAngle
from nav_msgs.msg import Odometry

#Global Variables
velocity = 0
phi = 0
l = 0.27
x = 0
y = 0
theta = 0
res = Odometry()
res.header.frame_id = "map"
res.child_frame_id = "base_link"

#Callback Functions for Subscriber Nodes
def callback_velocity(data):
    global velocity
    velocity = data.value

def callback_phi(data):
    global phi
    phi = data.value

def callback_map(data):
    global x
    global y
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
rospy.sleep(1)
t0 = rospy.get_time()
#print(x,y,theta)

#Odometry Calculation
def odometry_callback(event):
    global x
    global y
    global t0
    global res

    def get_velocities():
        global velocity
        global theta
        global phi
        velocity_x = velocity * np.cos(theta)
        velocity_y = velocity * np.sin(theta)
        velocity_theta = velocity / l * np.tan(phi)
        return velocity_x, velocity_y, velocity_theta
    #print(get_velocities())
    rospy.sleep(1)
    t1 = rospy.get_time()
    delta_t = t1-t0
    res.pose.pose.position.x = x + delta_t * get_velocities()[0]
    res.pose.pose.position.y = y + delta_t * get_velocities()[1]
    res.pose.pose.orientation.w = theta + delta_t * get_velocities()[2]
    odometryPublisher.publish(res)

rospy.Timer(rospy.Duration(0.01), odometry_callback)
rospy.spin()