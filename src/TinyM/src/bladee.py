#!/usr/bin/env python3.10

import rospy
import time
import signal
import sys
from geometry_msgs.msg import Twist
from tinymovr.tee import init_tee
from tinymovr.config import create_device
from pynput import keyboard

import can
from canine import CANineBus

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


#!/usr/bin/env python3.10

import rospy
import time
import signal
import sys
from geometry_msgs.msg import Twist
from tinymovr.tee import init_tee
from tinymovr.config import create_device
from pynput import keyboard

import can
from canine import CANineBus

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

        # Initialize keyboard listener
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.keyboard_listener.start()

        # Initialize Twist message publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_vel_msg = Twist()

    def engage(self):
        self.tm1.reset()
        self.tm2.reset()
        
        self.tm1.controller.velocity_mode()
        time.sleep(2)
        self.tm2.controller.velocity_mode()
        time.sleep(2)

        signal.signal(signal.SIGINT, self.signal_handler)

    def on_key_press(self, key):
        if key == keyboard.Key.w:
            # Move forward
            self.cmd_vel_msg.linear.x = 0.5
        elif key == keyboard.Key.s:
            # Move backward
            self.cmd_vel_msg.linear.x = -0.5
        elif key == keyboard.Key.a:
            # Turn left
            self.cmd_vel_msg.angular.z = 1.0
        elif key == keyboard.Key.d:
            # Turn right
            self.cmd_vel_msg.angular.z = -1.0

        # Publish the Twist message
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def on_key_release(self, key):
        if key in [keyboard.Key.w, keyboard.Key.s]:
            # Stop forward/backward motion
            self.cmd_vel_msg.linear.x = 0.0
        elif key in [keyboard.Key.a, keyboard.Key.d]:
            # Stop turning
            self.cmd_vel_msg.angular.z = 0.0

        # Publish the updated Twist message
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def main(self):
        rospy.loginfo("Robot Motion Control Node started.")
        self.engage()  # Start the controllers
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
