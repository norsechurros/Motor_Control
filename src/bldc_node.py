#!/usr/bin/env python3.10

import rospy
import can
from tinymovr.tee import init_tee
from tinymovr.config import get_bus_config, create_device
from canine import CANineBus
from geometry_msgs.msg import Twist

#as of now when i run this file, i get the 'can.exceptions.CanInitializationError: No active interface found
#' error , working on fixing that

#the configs file is just meant to be run once only when some changes are made

class TinyM:
    
    def __init__(self):
        
        self.params = get_bus_config(["canine", "slcan"])
        self.bus = can.Bus(interface="canine", bitrate=1000000)
        self.params["bitrate"] = 100000 
        init_tee(can.Bus(**self.params))
        self.tm = create_device(node_id=1)
        
        rospy.init_node('motion_control')
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,self.cmd_vel_clbk)
        self.rate = rospy.Rate(10)  # Update rate in Hz
    
    def cmd_vel_clbk(self,msg):
        
        desired_velocity = msg.linear.x  # Adjust this as needed
        
        self.tm.controller.calibrate()
        self.tm.controller.velocity_mode()
        self.tm.controller.vel_setpoint = int(desired_velocity * 100)  # Convert to appropriate units
        
    
    def main(self):
        rospy.loginfo("Robot Motion Control Node started.")
        while not rospy.is_shutdown():
            self.rate.sleep()
            
        
if __name__ == '__main__':
    try:
        controller = TinyM()
        controller.main()
    except rospy.ROSInterruptException:
        pass
            
            
        

        
