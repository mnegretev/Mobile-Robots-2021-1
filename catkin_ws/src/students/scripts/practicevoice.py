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
from std_msgs.msg import String

NAME = "OLIVAS_DIAZ"

pub_point = None

def callback1(req):
    point =  PoseStamped()
    #print(req.data)
    if(req.data=="GOTO LIVINGROOM"):
        point.pose.position.x=2.3;
        point.pose.position.y=-0.12;
        print(point)
        #point.point.x = 3
        #point.point.y = 3
        pub_point.publish(point)
    if(req.data=="GOTO OUTSIDE"):
        point.pose.position.x=0.0;
        point.pose.position.y=0.0;
        print(point)
        #point.point.x = 3
        #point.point.y = 3
        pub_point.publish(point)
    #pub_cmd_vel.publish(calculate_control(Pr[0],Pr[1],Pr[2],local_goal[0],local_goal[1]))
    #pub_cmd_vel.publish(cmd_vel1)


def main():
    global pub_point
    print "PRACTICE VOICE - " + NAME
    rospy.init_node("practicevoice")
    #rospy.wait_for_service('/static_map')
    pub_point = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber('recognized',String, callback1)
    #rospy.ServiceProxy('/navigation/path_planning/a_star_search'  , GetPlan, callback)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
