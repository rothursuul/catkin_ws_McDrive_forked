import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan

class LaneChange:

    def __init__(self):
        rospy.init_node("current_lane", anonymous=False)
        self.rate = rospy.Rate(10)
        self.lane_publisher = rospy.Publisher("/current_lane", Int32, queue_size=10)
        self.laser_scan_sub = rospy.Subscriber("/sensors/rplidar/scan", LaserScan, on_laser_scan, queue_size=10)

        self.lane = 0        
        
        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_laser_scan(self):
        self.scan = msg

        self.ranges = self.scan.ranges
        self.angle = self.scan.angle_increment
        

        
        lane_publisher.publish(data = self.lane)
