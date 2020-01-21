import rospy
import numpy as np
from autominy_msgs.msg import SpeedCommand
import tf.transformations
import math

class SpeedPID:

    def __init__(self, target_speed):
        rospy.init_node("speed_pid")
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.speed_sub = rospy.Subscriber("/sensors/speed", SpeedCommand, self.on_speed_change, queue_size=1)

        self.current_speed = 0.0
        self.rate = rospy.Rate(100)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.on_control)

        self.kp = 0.7
        self.ki = 0.09
        self.kd = 0.09
        self.min_i = -1.0
        self.max_i = 1.0

        self.integral_error = 0.0
        self.last_error = 0.0

        #target speed
        self.target_speed = target_speed

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_speed_change(self, msg):
        self.current_speed = msg.value

    def on_control(self, tmr):
        if tmr.last_duration is None:
            dt = 0.01
        else:
            dt = (tmr.current_expected - tmr.last_expected).to_sec()


        if self.target_speed is None:
            print("Desired Target Speed is None!!")
            return

        error = np.abs(self.target_speed - self.current_speed)
        print(self.current_speed)
        self.integral_error += error * dt
        self.integral_error = max(self.min_i, self.integral_error)
        self.integral_error = min(self.max_i, self.integral_error)

        derivative_error = (error - self.last_error) / dt
        self.last_error = error

        pid_output = self.kp * error + self.kd * derivative_error
        # print(pid_output)
        speed_msg = SpeedCommand()
        speed_msg.value = pid_output
        speed_msg.header.frame_id = "base_link"
        speed_msg.header.stamp = rospy.Time.now()

        self.speed_pub.publish(speed_msg)



if __name__ == "__main__":
    t = input("Target Speed: ")
    SpeedPID(t)
