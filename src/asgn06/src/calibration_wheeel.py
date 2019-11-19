import rospy

from autominy_msgs.msg import SteeringFeedback, SpeedCommand, NormalizedSteeringCommand, Tick
from nav_msgs.msg import Odometry

speed = SpeedCommand()
steering = NormalizedSteeringCommand()
rospy.init_node("calibration_wheeel", anonymous=False)
steeringPublisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
speedPublisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
rospy.sleep(1)


currentPosition_x = 0
currentPosition_y = 0
currentPosition_z = 0
def stop_car():
    speed.value = 0.0
    speedPublisher.publish(speed)

def callback_position(data):
    global currentPosition_x
    global currentPosition_y
    global currentPosition_z
    currentPosition_x = data.pose.pose.position.x
    currentPosition_y = data.pose.pose.position.y
    currentPosition_z = data.pose.pose.position.z

def get_current_position():
    rospy.Subscriber("/communication/gps/6", Odometry, callback_position)
    rospy.sleep(1)
get_current_position()
print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)

stop = 0
stop_position = currentPosition_x + 1
while currentPosition_x < stop_position:
    print("x:", currentPosition_x, "y:", currentPosition_y, "z:", currentPosition_z)
    steering.value = 0
    steeringPublisher.publish(steering)
    speed.value = 0.1
    speedPublisher.publish(speed)
    stop = stop + 1
#stop_car()