#!/usr/bin/env python3.10

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

AXLE_LENGTH = 0.67 # distance between the left and right wheels
WHEEL_RADIUS = 0.19 # radius of each wheel
NO_OF_POLE = 4


class TinyM:
    
    
    def __init__(self):
        
        params = get_bus_config()
        params["interface"]="slcan"
        params["bitrate"] = 1000000
        params["channel"]="/dev/ttyACM1"
        init_tee(can.Bus(**params))
        self.tm = create_device(node_id=1)
        time.sleep(0.1)

        self.tm.encoder.type = 1
        time.sleep(0.1)
        print(self.tm.encoder.type)
        time.sleep(0.1)

        self.tm.encoder.bandwidth = 1500
        time.sleep(0.1)
        print(self.tm.encoder.bandwidth)
        time.sleep(0.1)


        self.tm.motor.pole_pairs = 4
        time.sleep(0.1)
        print(self.tm.motor.pole_pairs)
        time.sleep(0.1)

        self.tm.controller.position.p_gain = 0.01 #best working as of now
        time.sleep(0.1)
        print(self.tm.controller.position.p_gain)
        time.sleep(0.1)
        self.tm.controller.velocity.p_gain = 0.01
        time.sleep(0.1)
        print(self.tm.controller.velocity.p_gain)
        time.sleep(0.1)
        print(self.tm.errors)
        rospy.init_node('motion_control')
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,self.cmd_vel_clbk)
        self.rate = rospy.Rate(10)  # Update rate in Hz
        
        self.tm.controller.velocity_mode()
        time.sleep(2)
        print(self.tm.controller.mode)
        time.sleep(2)
        
        self.tm.controller.velocity.setpoint = 200
        time.sleep(7)
        print(self.tm.controller.velocity.setpoint)

        self.tm.controller.idle()
    
    def cmd_vel_clbk(self,msg):
        
        left_w_vel = msg.linear.x - (msg.angular.x*AXLE_LENGTH)
        right_w_vel = msg.linear.x + (msg.angular.x*AXLE_LENGTH)
        
        left_w_rpm = (left_w_vel/2*3.14*WHEEL_RADIUS)*60
        right_w_rpm = -(right_w_vel/2*3.14*WHEEL_RADIUS)*60
        
        self.tm.controller.velocity.setpoint = (left_w_rpm/60)* 24*60
        
    def cmd_vel_sub(self):
        self.tm.controller.velocity.setpoint = 0
        
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_clbk)

        
        
    def main(self):
        rospy.loginfo("Robot Motion Control Node started.")
        while not rospy.is_shutdown():
            self.rate.sleep()
            
        
if __name__ == '__main__':
    try:
        controller = TinyM()
        controller.main()
        rospy.init_node('bldc_nodee', anonymous=True)

    except rospy.ROSInterruptException:
        pass
            
            
        
