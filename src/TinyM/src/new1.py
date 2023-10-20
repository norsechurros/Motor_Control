#!/usr/bin/env python3.10

import rospy
import time
import signal
import sys
import threading
from geometry_msgs.msg import Twist
from tinymovr.tee import init_tee
from tinymovr.config import create_device, get_bus_config
import can
from std_msgs.msg import Float64
import matplotlib.pyplot as plt  # Import Matplotlib

AXLE_LENGTH = 0.67
WHEEL_RADIUS = 0.19

class TinyM:

    def __init__(self):
        rospy.init_node('bldc_nodee', anonymous=True)

        params = get_bus_config()
        params["interface"] = "slcan"
        params["bitrate"] = 1000000
        params["channel"] = "/dev/ttyACM0"
        init_tee(can.Bus(**params))

        self.tm3 = create_device(node_id=3)
        self.tm2 = create_device(node_id=2)

        self.tm3.controller.calibrate()
        time.sleep(2)
        self.tm2.controller.calibrate()
        time.sleep(2)

        self.tm3.encoder.type = 1
        self.tm3.motor.pole_pairs = 4
        self.tm3.controller.position.p_gain = 0.007
        self.tm3.controller.velocity.p_gain = 0.012

        self.tm2.encoder.type = 1
        self.tm2.motor.pole_pairs = 4
        self.tm2.controller.position.p_gain = 0.007
        self.tm2.controller.velocity.p_gain = 0.012

        self.rate = rospy.Rate(10)

        # Create a lock for thread safety
        self.lock = threading.Lock()

        # Initialize lists for storing time and encoder velocity estimates
        self.time_data = []
        self.velocity_tm2_data = []
        self.velocity_tm3_data = []

    def engage(self):
        self.tm3.reset()
        self.tm2.reset()

        self.tm3.controller.velocity_mode()
        self.tm3.controller.velocity_setpoint = 0
        time.sleep(2)
        self.tm2.controller.velocity_mode()
        self.tm2.controller.velocity_setpoint = 0

        time.sleep(2)

        signal.signal(signal.SIGINT, self.signal_handler)

    def cmd_vel_clbk(self, msg):
        left_w_vel = msg.linear.x - (msg.angular.z * AXLE_LENGTH)
        right_w_vel = msg.linear.x + (msg.angular.z * AXLE_LENGTH)

        left_w_rpm = (left_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60
        right_w_rpm = -(right_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60

        self.tm3.controller.velocity.setpoint = -(right_w_rpm / 60) * 24 * 60
        self.tm2.controller.velocity.setpoint = (left_w_rpm / 60) * 24 * 60

        st1 = self.tm3.controller.state
        print(st1)

        st2 = self.tm2.controller.state
        print(st2)

        if st1 == 0 or st2 == 0:
            self.tm3.reset()
            self.tm2.reset()

    def encoder_pub(self):
        pub1 = rospy.Publisher('encoder_pos_tm1', Float64, queue_size=10)
        pub2 = rospy.Publisher('encoder_pos_tm2', Float64, queue_size=10)
        pub3 = rospy.Publisher('encoder_vel_tm2', Float64, queue_size=10)
        pub4 = rospy.Publisher('encoder_vel_tm3', Float64, queue_size=10)

        while not rospy.is_shutdown():
            with self.lock:
                enc_vel_estTM1 = self.tm3.encoder.velocity_estimate.magnitude
                enc_vel_estTM2 = self.tm2.encoder.velocity_estimate.magnitude
                enc_pos_estTM1 = self.tm3.encoder.position_estimate.magnitude
                enc_pos_estTM2 = self.tm2.encoder.position_estimate.magnitude

                pub1.publish(Float64(enc_pos_estTM1))
                pub2.publish(Float64(enc_pos_estTM2))
                pub3.publish(Float64(enc_vel_estTM1))
                pub4.publish(Float64(enc_vel_estTM2))

                # Append data to lists
                self.time_data.append(time.time())  # Store the current time
                self.velocity_tm2_data.append(enc_vel_estTM2)
                self.velocity_tm3_data.append(enc_vel_estTM1)

    def plot_velocity_vs_time(self):
        plt.figure()
        plt.plot(self.time_data, self.velocity_tm2_data, label='TM2 Velocity Estimate')
        plt.plot(self.time_data, self.velocity_tm3_data, label='TM3 Velocity Estimate')
        plt.xlabel('Time (s)')
        plt.ylabel('Encoder Velocity Estimate')
        plt.legend()
        plt.title('Encoder Velocity Estimates vs. Time')
        plt.grid(True)
        plt.show()

    def main(self):
        rospy.loginfo("Robot Motion Control Node started.")
        self.engage()
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_clbk)
        encoder_thread = threading.Thread(target=self.encoder_pub)
        encoder_thread.start()

        # Create a thread to periodically plot the data
        plot_thread = threading.Thread(target=self.plot_velocity_vs_time)
        plot_thread.start()

        while not rospy.is_shutdown():
            self.rate.sleep()

    def signal_handler(self, signum, frame):
        print("Stopping the program and idling the controller...")
        self.tm3.controller.idle()
        self.tm2.controller.idle()
        sys.exit(0)

if __name__ == '__main__':
    try:
        controller = TinyM()
        controller.main()
    except rospy.ROSInterruptException:
        pass
