#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 0 - THE PLATFORM ROS 
#
# Instructions:
# Write a program to move the robot forward until the laser
# detects an obstacle in front of it.
# Required publishers and subscribers are already declared and initialized.
#

import rospy
from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist

NAME = "Rangel_Navarro_Abraham_salvador"

def callback_laser_scan(msg):
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    #
    '''
    print('Recived laser scan with ' + str(len(msg.ranges)))
    print('Angle min: ' + str(msg.angle_min))
    print('Angle Increment: ' + str(msg.angle_increment))
    index = int((0-msg.angle_min)/msg.angle_increment)
    print('Index for 0 rad: ' + str(index))
    print('Distance at 0 [rad] : ' + str(index))
    print('Distance  at 0 [rad]: ' + str(msg.ranges[index]))
    '''
    index = int((0 - msg.angle_min)/msg.angle_increment)
   
    global obstacule_detected
    obstacule_detected = msg.ranges[index] < 1.0

    #if msg.ranges[index] < 1.0:
    #    print('Warning obstacule detected')
    #else:
    #    print('No risk of collision')
    return

def main():
    print "PRACTICE 00 - " + NAME
    rospy.init_node("practice00")
    rospy.Subscriber("/scan", LaserScan, callback_laser_scan)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    loop = rospy.Rate(10)
    

    global obstacule_detected
    obstacule_detected = True
    while not rospy.is_shutdown():
        #
        # TODO:
        # Declare a Twist message and assign the appropiate speeds:
        # Move forward if there is no obstacle in front and stop       otherwise.
        # Publish the message.
        #
        cmd_vel = Twist()
        if not obstacule_detected:
           cmd_vel.linear.x = 0.5
        else:
           cmd_vel.linear.x = 0
        pub_cmd_vel.publish(cmd_vel)
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
