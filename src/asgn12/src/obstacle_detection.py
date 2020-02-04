import rospy
import numpy as np
from map import Lane, Map


class obstacle_detection:

    def __init__(self, position, rplidar, current_lane):
        self.pose = position
        self.scan = rplidar
        self.lane = current_lane
        self.ranges = list(self.scan.ranges)
        #self.map = map

        print(self.ranges)

        self.rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.rate.sleep()

        

    def is_obstacle(self):

        current_position = np.array([self.pose.pose.pose.position.x, self.pose.pose.pose.position.y])
        orientation_angle = 2*np.arccos(self.pose.pose.pose.orientation.w)*np.sign(self.pose.pose.pose.orientation.z)
        transformation_matrix = np.array([[np.cos(orientation_angle), -np.sin(orientation_angle), 0, current_position[0]], [np.sin(orientation_angle), np.cos(orientation_angle), 0, current_position[1]], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        for r in range(len(self.ranges)):
            if self.ranges[r] == np.inf:
                self.ranges[r] = 1000
        
        print(self.ranges)

        scan_points_MapFrame = np.zeros((len(self.ranges),2))
        for r in range(len(self.ranges)):
            scan_point = np.array([self.ranges[r] * np.cos(self.scan.angle_min + r * self.scan.angle_increment), self.ranges[r] * np.sin(self.scan.angle_min + r * self.scan.angle_increment), 0, 1])
            scan_point_MapFrame = np.dot(transformation_matrix, scan_point)
            scan_points_MapFrame[r,:] = scan_point_MapFrame[:2]

        #print(scan_points_MapFrame)
        
        map = Map()
        closest_point_car, _ = map.lanes[self.lane].closest_point(current_position)
        #print(len(self.scan.ranges))
        closest_point_scan  = np.zeros((scan_points_MapFrame.shape[0],2))
        for i in range(closest_point_scan.shape[0]):
            closest_point_scan[i,:] = map.lanes[self.lane].closest_point(scan_points_MapFrame[i,:])[0]

        #print(closest_point_scan[2,1])
        print(closest_point_car[0,:])
        distances = np.zeros((len(self.ranges),2))
        for d in range(distances.shape[0]):
            distances[d,:] = np.sqrt((closest_point_scan[d,0] - closest_point_car[0,0])**2 + (closest_point_scan[d,1] - closest_point_car[0,1])**2)
        
        if np.any(np.any(distances < 0.15, axis=1)):
            obs = True
        else:
            obs = False
        print(distances)
        return obs
