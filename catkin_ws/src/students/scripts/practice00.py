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


NAME = "MARTINEZ FADUL JESUS"


def callback_laser_scan(msg):
    #
    # TODO:
    # Do something to detect if there is an obstacle in front of the robot.
    #
<<<<<<< HEAD
    #print("Received laser scan with: " + str(len(msg.ranges)))
    #print(str(msg.angle_min))
    #print(str(msg.angle_increment))
    index = int((0 - msg.angle_min)/msg.angle_increment)
    #print(str(msg.ranges[index])
    #if(msg.ranges[index] < 1.0)
    #    print("Obstacle detected)
    global obstacle_detected
    obstacle_detected = msg.ranges[index] < 1.0

    return
=======
    #print("Received laser scan with " + str(len(msg.ranges)))
    #print("Angle min: " + str(msg.angle_min))
    #print("Angle increment: " + str(msg.angle_increment))
    index = int((0 - msg.angle_min)/msg.angle_increment)
    #print("Index for 0 rad:" +  str(index))
    #print("Distance at 0 rad: " +  str(msg.ranges[index]))
    global obstacle_detected
    obstacle_detected = msg.ranges[index] < 1.0
    #if(msg.ranges[index] < 1.0):
    #    print("Warning! Obstacle detected")
    #else:
    #    print("No risk of collision")
    #return
>>>>>>> c3fe4591c16fca74a4c4050530c42bb69e5fe225


def main():
    print "PRACTICE 00 - " + NAME
    rospy.init_node("practice00")
    rospy.Subscriber("/scan", LaserScan, callback_laser_scan)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    loop = rospy.Rate(10)

    global obstacle_detected
    obstacle_detected = True

    while not rospy.is_shutdown():
        #
        # TODO:
        # Declare a Twist message and assign the appropiate speeds:
        # Move forward if there is no obstacle in front and stop otherwise.
        # Publish the message.
        #
        cmd_vel = Twist()
        if not obstacle_detected:
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