import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
import random

class LaneChange:

    def __init__(self):
        rospy.init_node("current_lane", anonymous=False)
        self.rate = rospy.Rate(10)
        self.lane_publisher = rospy.Publisher("/current_lane", Int32, queue_size=10)
        self.laser_scan_sub = rospy.Subscriber("/sensors/rplidar/scan", LaserScan, on_laser_scan, queue_size=10)

        self.lanes = [0,1]
        self.lane =  random.choice(self.lanes)
        
        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_laser_scan(self):
        self.scan = msg

        self.scan_points = np.zeros((len(self.scan.ranges),2))
        for r in range(len(self.scan.ranges)):
            self.scan_points[r,:] = [self.scan.ranges[r] * np.cos(self.scan.angle_min + r * self.scan.angle_increment), 
                                                        self.scan.ranges[r] * np.sin(self.scan.angle_min+ r * self.scan.angle_increment)]

        scan_points_MapFrame =     
        closest_point = 
        if np.any(np.any((scan_points_MapFrame - clostest_points) < 0.15, axis=1)):
            self.lane = list(self.lanes[i] for i in self.lanes if x[i] != self.lane)[0]
        else:
            self.lane = self.lane

            

        

        
        lane_publisher.publish(data = self.lane)
