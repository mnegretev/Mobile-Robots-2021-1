#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 4 - PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path using the gradient descend algorithm.
# Re-use the practice02 codes to implement the Dijkstra and A* algorithm.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import sys
import numpy
import heapq
import rospy
import copy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import String

NAME = "OLIVAS_DIAZ"

pub_point = None

def callback1(req):
    global incoming_call
    global going
    point =  PoseStamped()
    #print(req.data)
    if(going == False):
        point.pose.orientation.w=1;
        if(req.data=="GOTO LIVINGROOM"):
            point.pose.position.x=8.0;
            point.pose.position.y=4.2;
            #print(point)
            #point.point.x = 3
            #point.point.y = 3
            pub_point.publish(point)
        if(req.data=="GOTO KITCHEN"):
            point.pose.position.x=2.6;
            point.pose.position.y=3.5;
            #print(point)
            #point.point.x = 3
            #point.point.y = 3
            pub_point.publish(point)
        if(req.data=="GOTO BEDROOM"):
            point.pose.position.x=5.8;
            point.pose.position.y=2.0;
            #print(point)
            #point.point.x = 3
            #point.point.y = 3
            pub_point.publish(point)
        if(req.data=="GOTO OUTSIDE"):
            point.pose.position.x=0.1;
            point.pose.position.y=0.0;
            #print(point)
            #point.point.x = 3
            #point.point.y = 3
            pub_point.publish(point)
        incoming_call=True
    
    #pub_cmd_vel.publish(calculate_control(Pr[0],Pr[1],Pr[2],local_goal[0],local_goal[1]))
    #pub_cmd_vel.publish(cmd_vel1)

def callback_speed(vel):
    global incoming_call
    global going
    if(vel.linear.x!=0):
        going = True
    if(vel.linear.x==0 and incoming_call):
        going = False
        incoming_call = False

def main():
    global pub_point, incoming_call, going
    incoming_call = False
    going = False
    print "PRACTICE VOICE - " + NAME
    rospy.init_node("practicevoice")
    #rospy.wait_for_service('/static_map')
    pub_point = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('recognized',String, callback1)
    rospy.Subscriber('/cmd_vel', Twist, callback_speed)
    #rospy.ServiceProxy('/navigation/path_planning/a_star_search'  , GetPlan, callback)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
