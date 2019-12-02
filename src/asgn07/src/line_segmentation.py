import rospy
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
from matplotlib import image as mpimg
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

#Getting the infrared Image as a matrix
image = Image()
encoding = ""
bridge = CvBridge()
def callback(data):
    global image
    global encoding
    encoding = data.encoding
    image = data

#Getting the data from the bagfile
rospy.init_node("line_segmentation", anonymous=False)
rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, callback)
imagePublisher = rospy.Publisher("image_segmented", Image, queue_size=10)
rospy.sleep(1)

#Working on the Image
print(type(image))
print(encoding)
#print(image)
cv_image = bridge.imgmsg_to_cv2(image, encoding)
#cv.imshow("Image window", cv_image)
#cv.waitKey(0)

#Segmenting the image for white lines
ret, thresh_bin = cv.threshold(cv_image,254,255,cv.THRESH_BINARY)
#Crop
cropped_image = thresh_bin[170:320, 100:500]
# plt.imshow(cropped_image, 'gray')
# plt.show()

imagePublisher.publish(bridge.cv2_to_imgmsg(cropped_image, encoding))
#rospy.spin()

#---------------------------------------------------------------------
#RANSAC
def get_point_lib(A):
    res = []
    for x in range(len(A-1)):
        for y in range (len(A[x]-1)):
            if A[x,y] == 255:
                res.append([x,y])
    return np.array(res)

print(get_point_lib(cropped_image))
