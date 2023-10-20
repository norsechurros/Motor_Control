#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import math

# Constants for robot dimensions
AXLE_LENGTH = 0.67  # distance between the left and right wheels
WHEEL_RADIUS = 0.19  # radius of each wheel

class OdometryCalculator:

    def __init__(self):
        rospy.init_node('odometry_calculator', anonymous=True)
        self.rate = rospy.Rate(10)  # Update rate in Hz

        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_enc_pos = 0.0
        self.last_right_enc_pos = 0.0

        # Initialize ROS publishers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

        # Initialize ROS subscribers
        rospy.Subscriber('encoder_pos_tm1', Float64, self.left_encoder_callback)
        rospy.Subscriber('encoder_pos_tm2', Float64, self.right_encoder_callback)

    def left_encoder_callback(self, msg):
        # Calculate left wheel displacement since the last update
        left_enc_pos = msg.data
        delta_left_enc_pos = left_enc_pos - self.last_left_enc_pos
        self.last_left_enc_pos = left_enc_pos

        # Calculate linear and angular velocity of the left wheel
        left_wheel_velocity = delta_left_enc_pos * WHEEL_RADIUS
        angular_velocity = left_wheel_velocity / AXLE_LENGTH

        # Update robot's pose based on odometry
        delta_x = left_wheel_velocity * math.cos(self.theta)
        delta_y = left_wheel_velocity * math.sin(self.theta)
        delta_theta = angular_velocity

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

    def right_encoder_callback(self, msg):
        # Calculate right wheel displacement since the last update
        right_enc_pos = msg.data
        delta_right_enc_pos = right_enc_pos - self.last_right_enc_pos
        self.last_right_enc_pos = right_enc_pos

        # Calculate linear and angular velocity of the right wheel
        right_wheel_velocity = delta_right_enc_pos * WHEEL_RADIUS
        angular_velocity = right_wheel_velocity / AXLE_LENGTH

        # Update robot's pose based on odometry
        delta_x = right_wheel_velocity * math.cos(self.theta)
        delta_y = right_wheel_velocity * math.sin(self.theta)
        delta_theta = angular_velocity

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert yaw (theta) to quaternion
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # Publish odometry message
        self.odom_pub.publish(odom)

    def main(self):
        while not rospy.is_shutdown():
            self.publish_odometry()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        odometry_calculator = OdometryCalculator()
        odometry_calculator.main()
    except rospy.ROSInterruptException:
        pass
