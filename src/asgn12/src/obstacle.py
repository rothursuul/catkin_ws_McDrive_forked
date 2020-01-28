import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from autominy_msgs.msg import NormalizedSteeringCommand, SteeringCommand, SpeedCommand
import tf.transformations
import math
from map import Lane, Map


class ObstacleAvoidance:

    def __init__(self,nav):
        rospy.init_node("obstacle_avoidance")
        self.steering_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        self.localization_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.on_localization, queue_size=1)
        self.steering_sub = rospy.Subscriber("/control/steering", SteeringCommand, self.on_steering, queue_size=1)
        self.lane_sub = rospy.Subscriber("/current_lane", Int32, self.on_lane_change, queue_size=10)

        self.pose = Odometry()
        self.quat = []
        self.current_lane = None
        self.current_speed = 0.0
        self.rate = rospy.Rate(100)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.on_control) 

        self.kp = 4.0
        self.ki = 0.1
        self.kd = 0.1
        self.min_i = -1.0
        self.max_i = 1.0

        self.integral_error = 0.0
        self.last_error = 0.0

                self.desired_angle = None

        self.nav = nav

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_lane_change(self, msg):
            self.current_lane = msg.data

    def lane_nav(self):
        map = Map()
        current_position = np.array([self.pose.pose.pose.position.x, self.pose.pose.pose.position.y])

        if self.current_lane is None:
            print("No lane specified!")
            return

        lookahead_point = map.lanes[self.current_lane].lookahead_point(current_position, 0.5)
        _, _, yaw = tf.transformations.euler_from_quaternion(self.quat)
        diff = (lookahead_point[0] - current_position)
        delta = math.atan2(diff[0, 1], diff[0, 0])
        self.desired_angle = delta

    def on_localization(self, msg):
        self.pose = msg
        self.quat = [self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w]

    def on_steering(self, msg):
        self.desired_angle = msg.value

       def on_control(self, tmr):
        if tmr.last_duration is None:
            dt = 0.01
        else:
            dt = (tmr.current_expected - tmr.last_expected).to_sec()

        if self.nav:
            self.lane_nav()

        if self.desired_angle is None:
            print("Desired angle is none!!")
            return

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(self.quat)

        diff = self.desired_angle - yaw
        # normalize steering angles (keep between -pi and pi)
        error = math.atan2(math.sin(diff), math.cos(diff))

        self.integral_error += error * dt
        self.integral_error = max(self.min_i, self.integral_error)
        self.integral_error = min(self.max_i, self.integral_error)

        derivative_error = (error - self.last_error) / dt
        self.last_error = error

        pid_output = self.kp * error + self.kd * derivative_error + self.ki * self.integral_error
        steering_msg = NormalizedSteeringCommand()
        steering_msg.value = pid_output
        steering_msg.header.frame_id = "base_link"
        steering_msg.header.stamp = rospy.Time.now()

        self.steering_pub.publish(steering_msg)



if __name__ == "__main__":
    SteeringPID(nav=True)