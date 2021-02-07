#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# Final Project - Robots Moviles
# 
#
import sys
import rospy 
import tf

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sound_play.msg import SoundRequest
from std_msgs.msg import String

NAME = "ARGUELLES_MACOSAY"

confirm=0
actual_pos=1
place=0
pos_ant=0
#####################################
#Definir posiciones(coordenadas) de los lugares
#####################################
#ENTRANCE(0,0) - es la posicion casi inicial por defecto del robot
#BEDROOM(6,0)
#LIVINGROOM(5,5)
#CORRIDOR(3,0)
#KITCHEN(8,5)
def position(order):
    dir= PoseStamped()
    global confirm
    if (order==1):
        robot_talk("going to the entrance")
        posx,posy=(0,0)
        confirm=0
    elif (order==2):
        robot_talk("going to the bedroom")
        posx,posy=(6,0)
        confirm=0
    elif (order==3):
        robot_talk("going to the livingroom")
        posx,posy=(5,5)
        confirm=0
    elif (order==4):
        robot_talk("going to the corridor")
        posx,posy=(3,0)
        confirm=0
    elif (order==5):
        robot_talk("going to the kitchen")
        posx,posy=(8,5)
        confirm=0
    dir.pose.position.x=posx
    dir.pose.position.y=posy
    dir.pose.orientation.w=0
    publishing_dir.publish(dir)
    global pos_ant
    pos_ant=posx,posy
    print("goal:"+ str(pos_ant))
    return

def listening(order):
    order_received=order.data
    global confirm
    global actual_pos
    global place
    if(confirm == 0):
        if (order_received=="ROBOT GO TO THE ENTRANCE"):
            robot_talk("did you said go to the entrance")
            place=1
            confirm=1
        elif (order_received=="ROBOT GO TO THE BEDROOM"):
            robot_talk("did you said go to the bedroom")
            place=2
            confirm=1
        elif (order_received=="ROBOT GO TO THE LIVINGROOM"):
            robot_talk("did you said go to the livingroom")
            place=3
            confirm=1
        elif (order_received=="ROBOT GO TO THE CORRIDOR"):
            robot_talk("did you said go to the corridor")
            place=4
            confirm=1
        elif (order_received=="ROBOT GO TO THE KITCHEN"):
            robot_talk("did you said go to the kitchen")
            place=5
            confirm=1
    elif (confirm==1):
        if(order_received=="YES"):
            if (place==actual_pos):
                robot_talk("im already in the place")
                confirm=0
            else:
                position(place)
                actual_pos=place
                confirm=0
        elif(order_received=="NO"):
            robot_talk("sorry try again i didnt listen well")
            confirm=0
    else:
        robot_talk("I didn't recognice the voice command")

def robot_talk(to_say):
    print(to_say)
    Robot_voice=SoundRequest()
    Robot_voice.sound= -3
    Robot_voice.volume= 1
    Robot_voice.command= 1
    Robot_voice.arg= to_say
    publishing_robot_voice.publish(Robot_voice)

def main():
    global publishing_robot_voice
    global publishing_dir
    print ("PROJECT " + NAME)
    rospy.init_node("project")
    publishing_dir=rospy.Publisher('/move_base_simple/goal', PoseStamped,queue_size=10)
    publishing_robot_voice=rospy.Publisher('/robotsound',SoundRequest,queue_size=10)
    rospy.Subscriber("/recognized",String,listening)
    rospy.Rate(50)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
