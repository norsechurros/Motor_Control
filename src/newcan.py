#!/usr/bin/env python3.10

import can
from tinymovr.tee import init_tee
from canine import CANineBus

from tinymovr.config import get_bus_config, create_device

params = get_bus_config(['canine'])
params["bitrate"] = 1000000

bus = can.Bus(interface="canine", bitrate=1000000)

init_tee(can.Bus(**params))
tm = create_device(node_id=1)

tm.controller.calibrate()

tm.controller.velocity_mode()
tm.controller.vel_setpoint = 1000