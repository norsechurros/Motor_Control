#!/usr/bin/env python3.10

import time

from tinymovr.tee import init_tee
from tinymovr.config import  create_device

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


params = get_bus_config()
params["interface"]="slcan"
params["bitrate"] = 1000000
params["channel"]="/dev/ttyACM0"
init_tee(can.Bus(**params))
tm = create_device(node_id=1)
time.sleep(0.1)



tm.encoder.type = 1
time.sleep(0.1)
tm.encoder.bandwidth = 1500
time.sleep(0.1)
tm.motor.pole_pairs = 4
time.sleep(0.1)
tm.save_config()
time.sleep(0.1)
tm.reset() 
time.sleep(0.1)

tm.controller.calibrate()

time.sleep(1)
print(tm.calibrated)
time.sleep(0.1)

tm.controller.position.p_gain = 0.007 #best working as of now
time.sleep(0.1)

tm.controller.velocity.p_gain = 0.07
time.sleep(0.1)


tm.controller.velocity_mode()
time.sleep(2)
tm.controller.vel_setpoint = 200
time.sleep(5)



