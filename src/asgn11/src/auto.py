import rospy
from nav_msgs.msg import Odometry
#IMPORT MAP FROM STEFAN

x,y,z,w = 0.0

def callback_map(data):
    global x
    global y
    global z
    global w
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w

rospy.init_node("auto", anonymous=False)
rospy.Subscriber("/sensors/localization/filtered_map", Odometry, callback_map)
rospy.sleep(0)

