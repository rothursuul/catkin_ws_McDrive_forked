import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import random
from map import Map, Lane

class LaneChange:

    def __init__(self):
        rospy.init_node("current_lane", anonymous=False)
        self.rate = rospy.Rate(10)
        self.lane_publisher = rospy.Publisher("/current_lane", Int32, queue_size=10)
        self.laser_scan_sub = rospy.Subscriber("/sensors/rplidar/scan", LaserScan, self.laser_scan, queue_size=10)
        self.localization_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.localization, queue_size=1)

        self.rate = rospy.Rate(100)
        self.scan = LaserScan()
        self.pose = Odometry()
        self.lanes = [0,1]
        self.lane =  random.choice(self.lanes)

        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.on_control) 
     
        while not rospy.is_shutdown():
            self.rate.sleep()

    def laser_scan(self, msg):
        self.scan = msg
    
    def localization(self, msg):
        self.pose = msg

    def is_obstacle(self):

        current_position = np.array([self.pose.pose.pose.position.x, self.pose.pose.pose.position.y])
        orientation_angle = 2*np.arccos(self.pose.pose.pose.orientation.w)*np.sign(self.pose.pose.pose.orientation.z)
        transformation_matrix = np.array([[np.cos(orientation_angle), -np.sin(orientation_angle), 0, current_position[0]], [np.sin(orientation_angle), np.cos(orientation_angle), 0, current_position[1]], [0, 0, 1, 0], [0, 0, 0, 1]])
             
        scan_points_MapFrame = np.zeros((len(self.ranges),2))
        for r in range(len(self.ranges)):
            scan_point = np.array([self.ranges[r] * np.cos(self.scan.angle_min + r * self.scan.angle_increment), self.ranges[r] * np.sin(self.scan.angle_min + r * self.scan.angle_increment), 0, 1])
            scan_point_MapFrame = np.dot(transformation_matrix, scan_point)
            scan_points_MapFrame[r,:] = scan_point_MapFrame[:2]

        map = Map()
        closest_point_car, _ = map.lanes[self.lane].closest_point(current_position)
        closest_point_scan  = np.zeros((scan_points_MapFrame.shape[0],2))
        for i in range(closest_point_scan.shape[0]):
            closest_point_scan[i,:] = map.lanes[self.lane].closest_point(scan_points_MapFrame[i,:])[0]

        distances = np.zeros((len(self.ranges),2))
        for d in range(distances.shape[0]):
            distances[d,:] = np.sqrt((closest_point_scan[d,0] - closest_point_car[0,0])**2 + (closest_point_scan[d,1] - closest_point_car[0,1])**2)
        
        if np.any(np.any(distances < 0.15, axis=1)):
            self.lane = list(self.lanes[i] for i in self.lanes if self.lanes[i] != self.lane)[0]
        else:
            self.lane = self.lane
               
        self.lane_publisher.publish(data = self.lane)        
        
    
if __name__ == "__main__":
    LaneChange()
