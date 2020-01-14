import rospy
import numpy as np
#import scipy as scipy
from visualization_msgs.msg import Marker


lane1 = np.load("./lane1.npy")
lane2 = np.load("./lane2.npy")

print(len(lane1))
print(len(lane2))


rospy.init_node("test", anonymous=False)
spline = rospy.Publisher("spline", Marker, queue_size=100)
rospy.sleep(0)
marker = Marker()
spline.publish(marker)
rospy.spin()