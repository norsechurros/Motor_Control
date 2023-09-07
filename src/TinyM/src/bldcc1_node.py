#!/usr/bin/env python3.10

import rospy
import time
import signal
import sys
import threading  # Import the threading module
from geometry_msgs.msg import Twist
from tinymovr.tee import init_tee
from tinymovr.config import create_device,get_bus_config
import can
from std_msgs.msg import Float64

AXLE_LENGTH = 0.67  # distance between the left and right wheels
WHEEL_RADIUS = 0.19  # radius of each wheel

class TinyM:

    def __init__(self):
        rospy.init_node('bldc_nodee', anonymous=True)

        params = get_bus_config()
        params["interface"] = "slcan"
        params["bitrate"] = 1000000
        params["channel"] = "/dev/ttyACM0"
        init_tee(can.Bus(**params))

        self.tm1 = create_device(node_id=1)
        self.tm2 = create_device(node_id=2)

        self.tm1.encoder.type = 1
        self.tm1.motor.pole_pairs = 4
        self.tm1.controller.position.p_gain = 0.007
        self.tm1.controller.velocity.p_gain = 0.07

        self.tm2.encoder.type = 1
        self.tm2.motor.pole_pairs = 4
        self.tm2.controller.position.p_gain = 0.007
        self.tm2.controller.velocity.p_gain = 0.07

        self.rate = rospy.Rate(10)

        # Create a lock for thread safety
        self.lock = threading.Lock()

    def engage(self):
        self.tm1.reset()
        self.tm2.reset()

        self.tm1.controller.velocity_mode()
        time.sleep(2)
        self.tm2.controller.velocity_mode()
        time.sleep(2)

        signal.signal(signal.SIGINT, self.signal_handler)

    def cmd_vel_clbk(self, msg):
        left_w_vel = msg.linear.x - (msg.angular.z * AXLE_LENGTH)
        right_w_vel = msg.linear.x + (msg.angular.z * AXLE_LENGTH)

        left_w_rpm = (left_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60
        right_w_rpm = -(right_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60

        self.tm1.controller.velocity.setpoint = (right_w_rpm / 60) * 24 * 60
        self.tm2.controller.velocity.setpoint = (left_w_rpm / 60) * 24 * 60

    def encoder_pub(self):
        pub1 = rospy.Publisher('encoder_vel_tm1', Float64, queue_size=10)
        pub2 = rospy.Publisher('encoder_vel_tm2', Float64, queue_size=10)

        while not rospy.is_shutdown():
            with self.lock:  # Acquire the lock to safely access self.tm1 and self.tm2
                enc_vel_estTM1 = self.tm1.encoder.velocity_estimate.magnitude
                enc_vel_estTM2 = self.tm2.encoder.velocity_estimate.magnitude

            pub1.publish(Float64(enc_vel_estTM1))
            pub2.publish(Float64(enc_vel_estTM2))

    def main(self):
        rospy.loginfo("Robot Motion Control Node started.")
        self.engage()  # Start the controllers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_clbk)

        # Create a thread to publish encoder data
        encoder_thread = threading.Thread(target=self.encoder_pub)
        encoder_thread.start()

        while not rospy.is_shutdown():
            self.rate.sleep()

    def signal_handler(self, signum, frame):
        print("Stopping the program and idling the controller...")
        self.tm1.controller.idle()
        self.tm2.controller.idle()
        sys.exit(0)

if __name__ == '__main__':
    try:
        controller = TinyM()
        controller.main()
    except rospy.ROSInterruptException:
        pass
