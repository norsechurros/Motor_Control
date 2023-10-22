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

class configs:
    
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
