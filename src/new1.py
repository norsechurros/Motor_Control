#!/usr/bin/env python3.10


import can
from tinymovr.tee import init_tee
from canine import CANineBus

from tinymovr.config import get_bus_config, create_device

can.rc['interface'] = 'canine'
can.rc['channel'] = 'vcan0'
can.rc['bitrate'] = 1000000
from can.interface import Bus

bus = Bus()

bus = can.interface.Bus(bustype='canine', channel='vcan0', bitrate=500000)

tm = create_device(node_id=1)

tm.controller.calibrate()

tm.controller.velocity_mode()
tm.controller.vel_setpoint = 1000