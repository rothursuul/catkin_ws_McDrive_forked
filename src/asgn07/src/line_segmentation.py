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
<<<<<<< HEAD
#Crop
cropped_image = thresh_bin[170:320, 100:500]
# plt.imshow(cropped_image, 'gray')
# plt.show()

imagePublisher.publish(bridge.cv2_to_imgmsg(cropped_image, encoding))
#rospy.spin()

#---------------------------------------------------------------------

#Setting upper and lower part to zero and showing the resulting image
thresh_bin[:150,:] = 0
thresh_bin[320:,:] = 0
plt.imshow(thresh_bin, 'gray')
plt.show()

imagePublisher.publish(bridge.cv2_to_imgmsg(cv_image, encoding))
rospy.spin()

#RANSAC
def linear_fit(p1,p2):
	m = (p2[1]-p1[1])/(p2[0]-p1[0])
	b = p1[1]-m*p1[0]
	return m,b

N = 1e10
sample_count = 0
while N > sample_count:
	#choosing random sample and computing linear fit parameters
	num_points = len(white_pixels)
	idx1 = np.random.randint(0,num_points)
	idx2 = np.random.randint(0,num_points)
	p1 = white_pixels[idx1]
	p2 = white_pixels[idx2]
	m,b = linear_fit(p1,p2)

	#number of inliers
	t = np.sqrt(3.84)*std_dev
	y_model = m*white_pixels[:,0]+b
	num_inliers = 0
	for i in range(num_points):
		if y_model[i] < white_pixels[1,i] + t:
			num_inliers +=1

	#recomputing N
	s = 2
	p = 0.95
	e = 1-(num_inliers/num_points)
	N = np.log(1-p)/np.log(1-(1-e)**s)
	sample_count +=1

