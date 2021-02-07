#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# Final Project - Robots Moviles
# 
#
import sys
import rospy
import numpy 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

NAME = "ARGUELLES_MACOSAY"


def main():
    global pub_cmd_vel, loop, listener
    print "PROJECT - " + NAME
    rospy.init_node("project")
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
