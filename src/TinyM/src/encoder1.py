#!/usr/bin/env python3.10

import rospy
import math
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
import tf.transformations as tf

class EncoderToOdometry:

    def __init__(self):
        rospy.init_node('encoder_to_odom', anonymous=True)
        self.enc_vel_tm1 = 0.0
        self.enc_vel_tm2 = 0.0
        self.prev_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.enc_sub_tm1 = rospy.Subscriber('encoder_vel_tm1', Float64, self.enc_tm1_callback)
        self.enc_sub_tm2 = rospy.Subscriber('encoder_vel_tm2', Float64, self.enc_tm2_callback)

        self.odom = Odometry()
        
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.pose = Pose()
        self.twist = Twist()

    def enc_tm1_callback(self, data):
        self.enc_vel_tm1 = data.data

    def enc_tm2_callback(self, data):
        self.enc_vel_tm2 = data.data

    def calculate_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        # Calculate linear and angular velocity from encoder data
        linear_vel = (self.enc_vel_tm1 + self.enc_vel_tm2) / 2.0
        angular_vel = (self.enc_vel_tm2 - self.enc_vel_tm1) / 0.67  # Assuming axle length is 0.67

        # Calculate distance traveled
        linear_dist = linear_vel * dt
        angular_dist = angular_vel * dt

        # Update pose
        self.pose.position.x += linear_dist * math.cos(self.pose.orientation.z)
        self.pose.position.y += linear_dist * math.sin(self.pose.orientation.z)

        # Update quaternion (orientation)
        quaternion = tf.quaternion_from_euler(0, 0, self.pose.orientation.z + angular_dist)
        self.pose.orientation = Quaternion(*quaternion)

        # Update twist
        self.twist.linear.x = linear_vel
        self.twist.angular.z = angular_vel

        # Update header
        self.odom.header.stamp = current_time

        self.prev_time = current_time

    def publish_odometry(self):
        self.calculate_odometry()
        self.odom.pose.pose = self.pose
        self.odom.twist.twist = self.twist
        self.odom_pub.publish(self.odom)

    def main(self):
        rospy.loginfo("Encoder to Odometry Node started.")
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            self.publish_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        encoder_to_odom = EncoderToOdometry()
        encoder_to_odom.main()
    except rospy.ROSInterruptException:
        pass
