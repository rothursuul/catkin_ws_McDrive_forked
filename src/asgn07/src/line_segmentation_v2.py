from __future__ import division
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
	image = data
	encoding = data.encoding

#Getting the data from the bagfile
rospy.init_node("line_segmentation", anonymous=False)
rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, callback)
imagePublisher = rospy.Publisher("image_segmented", Image, queue_size=10)
rospy.sleep(1)

#Working on the Image
#print(type(image))
#print(encoding)
#print(image)
cv_image = bridge.imgmsg_to_cv2(image, encoding)
#cv.imshow("Image window", cv_image)
#cv.waitKey(0)

#Segmenting the image for white lines
ret, thresh_bin = cv.threshold(cv_image,254,255,cv.THRESH_BINARY)
#Crop
cropped_image = thresh_bin[170:320, 100:500]

#plt.imshow(cropped_image, 'gray')
#plt.show()

plt.imshow(cropped_image, 'gray')
plt.show()


imagePublisher.publish(bridge.cv2_to_imgmsg(cropped_image, encoding))
#rospy.spin()

#---------------------------------------------------------------------

#Setting upper and lower part to zero and showing the resulting image
cropped_image = thresh_bin[170:320, 100:500]
#plt.imshow(cropped_image, 'gray')
#plt.show()

imagePublisher.publish(bridge.cv2_to_imgmsg(cv_image, encoding))
#rospy.spin()

#RANSAC
def get_point_lib(A):
	res = []
	x_res = []
	y_res = []
	for x in range(len(A-1)):
		for y in range (len(A[x]-1)):
			if A[x,y] == 255:
				res.append([x,y])
				x_res.append(x)
				y_res.append(y)
	return np.array(res), np.array(x_res), np.array(y_res)

#print(get_point_lib(cropped_image))

def linear_fit(p1,p2):
	m = (p2[1]-p1[1])/(p2[0]-p1[0])
	b = p1[1]-m*p1[0]
	return m,b

white_pixel_map = get_point_lib(cropped_image)
white_pixels = white_pixel_map[0]
x_vals = white_pixel_map[1]
y_vals = white_pixel_map[2]
print(white_pixels[1])
print(x_vals[1])
N = 1e10
print (N)
s = 2
p = 0.95
sample_count = 0
check = 0
while N > sample_count:
	#choosing random sample and computing linear fit parameters
	num_points = len(white_pixels)
	idx1 = np.random.randint(0,num_points)
	idx2 = np.random.randint(0,num_points)
	p1 = white_pixels[idx1]
	p2 = white_pixels[idx2]
	m,b = linear_fit(p1,p2)

	#number of inliers

	sigma = np.sqrt(np.mean(np.abs(x_vals - x_vals.mean()) ** 2 + np.abs(y_vals - y_vals.mean()) ** 2)
	t = np.sqrt(3.84) * sigma
	y_model = m*white_pixels[:,0]+b
	num_inliers = 0
	for i in range(num_points):
		if y_model[i] < white_pixels[i,1] + t:
			num_inliers +=1


	#recomputing N
	s = 2
	p = 0.99
	#print(num_inliers)
	#print(num_points)
	#print(float(num_inliers)/num_points)
	e = 1-(float(num_inliers)/num_points)
	#print(e)
	#print(np.log(1-(1-e)**s))
	N = (np.log(1-p))/(np.log(1-(1-e)**s))
	sample_count +=1

print(m,b)
