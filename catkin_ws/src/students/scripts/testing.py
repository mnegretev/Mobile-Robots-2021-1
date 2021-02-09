#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# FINAL PROJECT - SERVICE ROBOT CONTROLED BY VOICE COMMANDS
#

import sys
import rospy
import tf
import math
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from geometry_msgs.msg import PoseStamped

pub_voice = None
pub_goal  = None
goal      = None
tol       = None
state     = 'idle'
loop      = None
global_g  = None
sub_voice = None
sub_synth = None
rospy.set_param("/navigation/path_planning/smoothing_beta",0.90)
rospy.set_param("/navigation/path_planning/smoothing_alpha",0.05)

def callback_voice_recognized(data):
    global state, global_g, loop
    destination = None
    rospy.loginfo(rospy.get_caller_id() + " Command detected: %s", data.data)
    #pub_voice.publish(-3,1,1.0,'New command recieved. Moving to','voice_kal_diphone')
    goal = PoseStamped()
  
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    
    phrase = data.data
    words = phrase.split()

    if (words[-1]=='KITCHEN'):
        goal.pose.position.x = 8.5
        goal.pose.position.y = 4.5
        goal.pose.position.z = 0.0
        destination = 'the kitchen'
    #elif (words[-1]=='BEDROOM'):
        #goal.pose.position.x = 5.2
        #goal.pose.position.y = 2.0
        #goal.pose.position.z = 0.0
        #destination = 'the bedroom'
    elif (words[-1]=='ROOM'):
        if (words[-2]=='LIVING'):
            goal.pose.position.x = 3.0
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            destination = 'the living room'
        elif (words[-2]=='DINING'):
            goal.pose.position.x = 4.0
            goal.pose.position.y = 4.0
            goal.pose.position.z = 0.0
            destination = 'the dining room'
    else:
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        destination = 'the initial position'
    
    pub_voice.publish(-3,1,1.0,'New command recieved. Moving to ' + destination,'voice_kal_diphone')
    pub_goal.publish(goal)
    state = 'moving'
    loop.sleep()

def callback_voice_synthesized(data):
    global state, loop
    #rospy.loginfo('The robot said something')
    if(data.arg=='Destination reached. Ready to recieve new commands'):
        state = 'idle'
        #rospy.loginfo('The robot arraived. Returning to initial state')
    loop.sleep()

def callback_voice_recognized_ignored(data):
    rospy.loginfo(rospy.get_caller_id() + " Command detected: %s", data.data)
    pub_voice.publish(-3,1,1.0,'Can not attend new command. Excecuting previous instruction','voice_kal_diphone')

def get_robot_pose(listener):
    try:
        (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
        robot_a = 2*math.atan2(rot[2], rot[3])
        if robot_a > math.pi:
            robot_a -= 2*math.pi
        return robot_x, robot_y, robot_a
    except:
        pass
    return [0,0,0]

def main():
    global pub_voice, pub_goal, tol, loop, state, global_g, sub_voice, sub_synth
    rospy.init_node("testing")
    loop = rospy.Rate(20)
    pub_voice = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    sub_voice = rospy.Subscriber('/recognized', String, callback_voice_recognized)
    sub_synth = rospy.Subscriber('/robotsound', SoundRequest, callback_voice_synthesized)
    listener = tf.TransformListener()
    #listener = tf.TransformListener()
    #rospy.spin()

    while not rospy.is_shutdown():
        if (state=='idle'):
            sub_voice.unregister()
            sub_voice = rospy.Subscriber('/recognized', String, callback_voice_recognized)
            #rospy.loginfo('initial state')
            loop.sleep()
        elif(state=='moving'):
            sub_voice.unregister()
            sub_voice = rospy.Subscriber('/recognized', String, callback_voice_recognized_ignored)
            #rospy.loginfo('first state detected')
            #state = 'initial'
            loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
