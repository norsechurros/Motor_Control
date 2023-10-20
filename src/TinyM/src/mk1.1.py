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
import matplotlib.pyplot as plt
from collections import deque
import time
import glob

AXLE_LENGTH = 0.76  # distance between the left and right wheels
WHEEL_RADIUS = 0.19  # radius of each wheel

class TinyM:
    def find_serial_devices(self, pattern):
        serial_devices = glob.glob(pattern)
        return serial_devices

    def __init__(self):
        rospy.init_node('bldc_nodee', anonymous=True)

        pattern = '/dev/ttyACM*'
        matching_devices = self.find_serial_devices(pattern)

        if matching_devices:
            # If matching devices are found, use the first one as the channel
            channel = matching_devices[0]
        else:
            # If no matching devices are found, use "/dev/ttyACM0" as a default
            channel = "/dev/ttyACM0"

        params = get_bus_config()
        params["interface"] = "slcan"
        params["bitrate"] = 1000000
        params["channel"] = channel
        init_tee(can.Bus(**params))

        self.tm3 = create_device(node_id=3)
        self.tm2 = create_device(node_id=2)
        
        self.tm3.reset()
        self.tm2.reset()

        self.tm3.encoder.type = 1
        self.tm3.motor.pole_pairs = 4
        self.tm3.controller.velocity.p_gain = 0.007
        self.tm3.controller.velocity.i_gain = 0.001
        self.tm3.save_config()
        self.tm3.reset()
        time.sleep(3)

        self.tm2.encoder.type = 1
        self.tm2.motor.pole_pairs = 4
        self.tm2.controller.velocity.p_gain = 0.007
        self.tm2.controller.velocity.i_gain = 0.001
        self.tm2.save_config()
        self.tm2.reset()
        time.sleep(3)

        self.rate = rospy.Rate(10)

        # Create a lock for thread safety
        self.lock = threading.Lock()

        self.enc_time_buffer = deque(maxlen=100)
        self.enc_vel_estTM1_buffer = deque(maxlen=100)
        self.enc_vel_estTM2_buffer = deque(maxlen=100)

    def plot_encoders(self):
        while not rospy.is_shutdown():
            with self.lock:
                enc_vel_estTM1 = self.tm3.encoder.velocity_estimate.magnitude
                enc_vel_estTM2 = self.tm2.encoder.velocity_estimate.magnitude
                enc_time = time.time()

            self.enc_vel_estTM1_buffer.append(enc_vel_estTM1)
            self.enc_vel_estTM2_buffer.append(enc_vel_estTM2)
            self.enc_time_buffer.append(enc_time)

            plt.figure(figsize=(8, 6))
            plt.plot(self.enc_time_buffer, self.enc_vel_estTM1_buffer, label='enc_vel_estTM1')
            plt.plot(self.enc_time_buffer, self.enc_vel_estTM2_buffer, label='enc_vel_estTM2')
            plt.xlabel('Time')
            plt.ylabel('Encoder Velocity')
            plt.legend()
            plt.grid(True)
            plt.title('Encoder Velocity vs. Time')
            plt.pause(0.01)

    def engage(self):
        self.tm3.controller.velocity_mode()
        self.tm3.controller.velocity_setpoint = 0
        time.sleep(2)
        self.tm2.controller.velocity_mode()
        self.tm2.controller.velocity_setpoint = 0
        time.sleep(2)
        signal.signal(signal.SIGINT, self.signal_handler)

    def cmd_vel_clbk(self, msg):
        st1 = self.tm3.controller.state
        st2 = self.tm2.controller.state
        
        if st1 == 0 or st2 == 0:
            print("Motors in error state, resetting motors")
            self.tm3.controller.velocity.setpoint = 0
            self.tm2.controller.velocity.setpoint = 0
            self.tm2.reset()
            self.tm3.reset()
            time.sleep(2)
             
        left_w_vel = msg.linear.x - (msg.angular.z * AXLE_LENGTH)
        right_w_vel = msg.linear.x + (msg.angular.z * AXLE_LENGTH)

        left_w_rpm = (left_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60
        right_w_rpm = -(right_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60

        self.tm3.controller.velocity.setpoint = -(right_w_rpm / 60) * 24 * 60
        self.tm2.controller.velocity.setpoint = (left_w_rpm / 60) * 24 * 60

        st3 = self.tm3.controller.warnings
        st4 = self.tm2.controller.warnings
        st5 = self.tm3.controller.errors
        st6 = self.tm2.controller.errors

        print("TM3 controller warnings:", st3)
        print("TM2 controller warnings:", st4)
        print("TM3 controller errors:", st5)
        print("TM2 controller errors:", st6)

    def encoder_pub(self):
        pub1 = rospy.Publisher('encoder_pos_tm1', Float64, queue_size=100)
        pub2 = rospy.Publisher('encoder_pos_tm2', Float64, queue_size=100)
        pub3 = rospy.Publisher('encoder_vel_tm1', Float64, queue_size=100)
        pub4 = rospy.Publisher('encoder_vel_tm2', Float64, queue_size=100)

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

    def main(self):
        rospy.loginfo("Robot Motion Control Node started.")
        self.engage()
        self.watchdog()
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_clbk)

        encoder_thread = threading.Thread(target=self.encoder_pub)
        encoder_thread.start()

        plot_thread = threading.Thread(target=self.plot_encoders)
        plot_thread.start()

        while not rospy.is_shutdown():
            self.rate.sleep()

    def watchdog(self):
        self.tm2.watchdog.timeout = 2
        self.tm3.watchdog.timeout = 2
    
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
