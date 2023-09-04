#!/usr/bin/env python3.10

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler

class OdometryCalculator:

    def __init__(self):
        rospy.init_node('odometry_calculator', anonymous=True)
        
        # Initialize encoder position estimates
        self.enc_pos_estTM1 = 0.0
        self.enc_pos_estTM2 = 0.0
        
        # Create a subscriber to the encoder velocity topic
        rospy.Subscriber('encoder_vel', Float64, self.encoder_callback)
        
        # Create an Odometry publisher
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.last_time = rospy.Time.now()

        # Initial robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def encoder_callback(self, data):
        # Update encoder position estimates
        if data._connection_header['topic'] == 'encoder_vel':
            if data._connection_header['callerid'].endswith('bldc_nodee'):
                self.enc_pos_estTM1 = data.data
            else:
                self.enc_pos_estTM2 = data.data

    def calculate_odometry(self):
        current_time = rospy.Time.now()
        delta_time = (current_time - self.last_time).to_sec()

        # Convert encoder data to linear displacement
        wheel_radius = WHEEL_RADIUS  # Replace with your wheel radius
        ticks_per_revolution = TICKS_PER_REVOLUTION  # Replace with your encoder's ticks per revolution

        delta_x = ((self.enc_pos_estTM1 + self.enc_pos_estTM2) / 2.0 / ticks_per_revolution) * (2 * 3.14 * wheel_radius)
        delta_y = 0.0  # Assuming no lateral movement (in a 2D plane)
        delta_theta = (self.enc_pos_estTM2 - self.enc_pos_estTM1) / ticks_per_revolution / AXLE_LENGTH

        # Update pose
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))

        # Set covariance values (if needed)
        odom_msg.pose.covariance = [0.0] * 36  # Replace with appropriate covariance values

        # Velocity (currently set to 0, you can calculate it if needed)
        odom_msg.twist.twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        # Set covariance values (if needed)
        odom_msg.twist.covariance = [0.0] * 36  # Replace with appropriate covariance values

        # Publish the Odometry message
        self.odom_pub.publish(odom_msg)

        self.last_time = current_time

    def main(self):
        rospy.loginfo("Odometry Calculator Node started.")
        while not rospy.is_shutdown():
            self.calculate_odometry()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        AXLE_LENGTH = 0.67  # distance between the left and right wheels
        WHEEL_RADIUS = 0.19  # radius of each wheel
        TICKS_PER_REVOLUTION = 1000  # Replace with your encoder's ticks per revolution
        
        odometry_calculator = OdometryCalculator()
        odometry_calculator.main()
    except rospy.ROSInterruptException:
        pass
