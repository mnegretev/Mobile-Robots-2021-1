#!/usr/bin/env python

import numpy as np
import glob
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from sound_play.msg import SoundRequest

#pub_coord = None
pub_pose   = None
pub_voice  = None



def to_order(word):
    global walk
    global finish
    localization = PoseStamped()

    #............................................................
    #
    # This is where the movement happens
    #
    #............................................................
    
    if (walk == False):
        if word=='GO TO BEDROOM':
            localization.pose.position.x = 8
            localization.pose.position.y = 0
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Walking to BEDROOM: located in  8,0'
        if word =='GO TO KITCHEN':
            localization.pose.position.x = 3
            localization.pose.position.y = 0
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Walking to KITCHEN: located in 1,1'
        if word=='GO TO LIVINGROOM':
            localization.pose.position.x = 3
            localization.pose.position.y = 6
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Walking to LIVINGROOM: located in  3,6'
        if word == "GO TO THE ENTRANCE":
            direccion.pose.position.x = 3
            direccion.pose.position.y = 0
            direccion.pose.orientation.w = 1
            pub_direccion.publish(direccion)
            print 'Walking to ENTRANCE: located in  3,0'
        finish=True
    if (walk==True):
        talk('I am still working on the previus command')
    return 0

def callback_function(msg):
    order = msg.data
    to_order(order)

def talk(phrase):
    #
    # This is where the robot "talks"
    #

    voice = SoundRequest()
    voice.sound = -3
    voice.command = 1
    voice.volume=1.0
    voice.arg=phrase
    pub_voice.publish(voice)

def callback_checar(msg):
    
    global walk
    global finish

    if(msg.linear.x!=0):
        walk = True
        print "Order received"

    if(msg.linear.x==0 and finish):
        finish  = False
        walk    = False
        phrase  = "Arrived to destination"
        print(phrase)
        talk(phrase)
    return


def main():
    global walk
    global finish
    global pub_pose, pub_voice

    pub_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 50)
    pub_voice = rospy.Publisher('/robotsound', SoundRequest, queue_size = 50)

    finish=False
    walk=False

    print(finish, walk)

    rospy.init_node('say', anonymous = True)
    rospy.Subscriber('/cmd_vel', Twist, callback_checar)
    rospy.Subscriber('/recognized',String,callback_function)
    
    loop = rospy.Rate(20)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
