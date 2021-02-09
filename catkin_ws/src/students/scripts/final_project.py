#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# Final_Project - CONTROL BY VOICE
#
# 

import sys
import rospy
import tf
from sound_play.msg import SoundRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

NAME = "NAJERA_GARCIA_EDUARDO"
pub_cmd_loc = None
loop        = None

def control_ins(command):
    global movement
    loc = PoseStamped()
    if not movement:
	loc.pose.orientation.w = 1
        if(command == "ROBOT GO TO THE LIVINGROOM"):
	    loc.pose.position.x = 3.0
	    loc.pose.position.y = 4.0
	    pub_cmd_loc.publish(loc)
	elif(command == "ROBOT GO TO THE BEDROOM"):
	    loc.pose.position.x = 8.0
	    loc.pose.position.y = 4.0
	    pub_cmd_loc.publish(loc)
	elif(command == "ROBOT GO TO THE KITCHEN"):
	    loc.pose.position.x = 3.0
	    loc.pose.position.y = 1.0
	    pub_cmd_loc.publish(loc)
        elif (command == "ROBOT GO TO THE ENTRANCE"):
            loc.pose.position.x = 0.0
	    loc.pose.position.y = 0.0
	    pub_cmd_loc.publish(loc)


def callback_ins(msg):
    order = msg.data
    control_ins(order)

def callback_goal(msg):
    global movement
    if(msg.linear.x != 0):
        movement = True
    elif(msg.linear.x == 0):
	movement = False

def main():
    global pub_cmd_loc, loop, movement
    movement = False
    print "FINAL PROJECT - " + NAME
    rospy.init_node("final_project")
    pub_cmd_loc = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('/recognized', String, callback_ins)
    rospy.Subscriber('/cmd_vel', Twist, callback_goal)
    loop = rospy.Rate(20)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
