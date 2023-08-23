#!/usr/bin/env python3.10

import can
from tinymovr.tee import init_tee
from tinymovr.config import get_bus_config, create_device
from canine import CANineBus

tm = create_device(node_id=1)


tm.encoder.type = 1
tm.encoder.bandwidth = 1500

tm.motor.pole_pairs = 4
tm.save_config()
tm.reset() 

tm.controller.position.p_gain = 0.007 #best working as of now
tm.controller.velocity.p_gain = 0.07