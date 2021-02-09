#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# FINAL PROJECT: CONTROLLING ROBOT WITH VOICE COMMANDS
#
import numpy as np
import glob
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sound_play.msg import SoundRequest

pub_pose = None
pub_voice   = None
NAME = "ARIAS_PELAYO"

#cuando el robot llega al punto meta publica un topico
def say(sentence):
    voice = SoundRequest()
    voice.sound = -3
    voice.command = 1
    voice.volume=1.0
    voice.arg = sentence
    pub_voice.publish(voice)

def callback_function(msg):
    command = msg.data
    move_to(command)

#checa la gramatica para saber a donde moverse
def move_to(word):

    localization = PoseStamped()
    global moving
    global recognized
    #parseamos y traducimos la palabra a coordenadas x,y
    if (moving == False):
        if (word=='GO TO KITCHEN'):
            localization.pose.position.x = 3
            localization.pose.position.y = 6
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Going to kitchen in : 3,6'
        elif (word =='GO TO LOUNGE'):
            localization.pose.position.x = 3
            localization.pose.position.y = 6
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Going to Lounge in : 9,6'
        elif (word =='GO TO BEDROOM'):
            localization.pose.position.x = 8
            localization.pose.position.y = 0
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Going to kitchen in : 7,-1'
        elif (word=='GO TO WALL'):
            localization.pose.position.x = 3
            localization.pose.position.y = 10
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Going to wall in : 3,10'
        recognized = True
    if (moving == True):
        say('The Robot is Busy')
    return 0


#checa si el robot ha llegado a su destino
def check_arrival(msg):
    
    global moving
    global recognized

    if(msg.linear.x!=0):# si la velocidad no es cero significa que el robot se mueve
        moving = True
    if(msg.linear.x == 0 and recognized):#si la velocidad es cero el robot ya ha llegado
        recognized = False
        moving = False
        sentence = "I have arrived to " + msg #mensaje + lugar
        say(sentence)
    return


def main():
    global moving
    global recognized
    global pub_pose, pub_voice
    print "Final Project - " + NAME
    pub_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pub_voice = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    recognized=False
    moving=False

    print(recognized, moving,'//')

    rospy.init_node('say', anonymous = True)
    rospy.Subscriber('/cmd_vel', Twist, check_arrival)
    rospy.Subscriber('/recognized',String,callback_function)
    loop = rospy.Rate(20)
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
