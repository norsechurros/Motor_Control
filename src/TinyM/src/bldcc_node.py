#!/usr/bin/env python3.10

import rospy
import time
import signal
import sys
from geometry_msgs.msg import Twist
from tinymovr.tee import init_tee
from tinymovr.config import create_device


import rospy
from tinymovr.tee import init_tee
from tinymovr.config import  create_device
import time

from geometry_msgs.msg import Twist

from canine import CANineBus

import can
from canine import CANineBus
import pkg_resources
import IPython
from traitlets.config import Config
from docopt import docopt

from tinymovr import init_tee, destroy_tee
from tinymovr.discovery import Discovery
from tinymovr.constants import app_name, base_node_name
from tinymovr.config import get_bus_config, configure_logging

import tinymovr
import signal
import sys


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
        self.tm1.controller.velocity.p_gain = 0.007

        
        self.tm2.encoder.type = 1
        self.tm2.motor.pole_pairs = 4
        
        self.tm2.controller.position.p_gain = 0.007
        self.tm2.controller.velocity.p_gain = 0.007

        self.rate = rospy.Rate(10) 

    def engage(self):
        self.tm1.controller.velocity.mode()
        time.sleep(2)
        self.tm2.controller.velocity.mode()
        time.sleep(2)

        signal.signal(signal.SIGINT, self.signal_handler)

    def cmd_vel_clbk(self, msg):
        left_w_vel = msg.linear.x - (msg.angular.z * AXLE_LENGTH)
        right_w_vel = msg.linear.x + (msg.angular.z * AXLE_LENGTH)

        left_w_rpm = (left_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60
        right_w_rpm = -(right_w_vel / (2 * 3.14 * WHEEL_RADIUS)) * 60

        self.tm1.controller.velocity.setpoint = (left_w_rpm / 60) * 24 * 80
        self.tm2.controller.velocity.setpoint = (right_w_rpm / 60) * 24 * 80

    def main(self):
        rospy.loginfo("Robot Motion Control Node started.")
        self.engage()  # Start the controllers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_clbk)
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
