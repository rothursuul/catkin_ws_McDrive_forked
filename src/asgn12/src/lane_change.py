import rospy
from std_msgs.msg import Int32

rospy.init_node("current_lane", anonymous=False)
rate = rospy.Rate(10)
lane_publisher = rospy.Publisher("/current_lane", Int32, queue_size=10)
rospy.sleep(1)
lane = input("Input new lane: ")
while not rospy.is_shutdown():
    lane_publisher.publish(data = lane)
    rate.sleep()