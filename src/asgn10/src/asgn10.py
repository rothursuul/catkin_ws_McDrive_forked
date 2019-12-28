import rospy
import numpy as np
#import scipy as scipy

lane1 = np.load("./lane1.npy")
lane2 = np.load("./lane2.npy")

print(len(lane1))
print(len(lane2))

#was heisst hier "arc length", ist da die distanz zum vorherigen Punkt gemeint?