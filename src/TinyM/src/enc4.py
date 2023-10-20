#!/usr/bin/env python3.10

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
import math

class WheelOdometry:

    def __init__(self):
        rospy.init_node('wheel_odometry', anonymous=True)
        self.enc_vel_tm1 = 0.0
        self.enc_vel_tm2 = 0.0
        self.prev_time = rospy.Time.now()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)  # Publish to /odom

        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.pose = Pose()
        self.twist = Twist()

        self.wheel_distance = 0.76  # Distance between wheels (assumed value)
        self.wheel_radius = 0.19  # Radius of each wheel (assumed value)

        rospy.Subscriber('encoder_vel_tm1', Float64, self.enc_tm1_callback)
        rospy.Subscriber('encoder_vel_tm2', Float64, self.enc_tm2_callback)

    def enc_tm1_callback(self, data):
        self.enc_vel_tm1 = data.data

    def enc_tm2_callback(self, data):
        self.enc_vel_tm2 = data.data

    def calculate_wheel_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        # Calculate linear and angular velocity from encoder velocity estimates
        linear_vel = (self.enc_vel_tm1 + self.enc_vel_tm2) / 2.0
        angular_vel = (self.enc_vel_tm2 - self.enc_vel_tm1) / self.wheel_distance

        # Calculate lateral velocity
        lateral_vel = angular_vel * self.wheel_distance / 2

        # Update pose
        self.pose.position.x += (linear_vel * dt)/500
        self.pose.position.y += (lateral_vel * dt)/500
        self.pose.position.z = 0  # Assuming no vertical movement

        # Update orientation (quaternion)
        delta_theta = angular_vel * dt
        quaternion = self.pose.orientation
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        new_quaternion = [0, 0, math.sin(delta_theta / 2), math.cos(delta_theta / 2)]
        updated_q = [a * b for a, b in zip(q, new_quaternion)]
        self.pose.orientation = Quaternion(*updated_q)

        # Update twist
        self.twist.linear.x = linear_vel
        self.twist.linear.y = lateral_vel
        self.twist.angular.z = angular_vel

        # Update header
        self.odom.header.stamp = current_time

        self.prev_time = current_time

    def publish_wheel_odometry(self):
        self.calculate_wheel_odometry()
        self.odom.pose.pose = self.pose
        self.odom.twist.twist = self.twist
        self.odom_pub.publish(self.odom)

    def main(self):
        rospy.loginfo("Wheel Odometry Node started.")
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            self.publish_wheel_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        wheel_odometry = WheelOdometry()
        wheel_odometry.main()
    except rospy.ROSInterruptException:
        pass
