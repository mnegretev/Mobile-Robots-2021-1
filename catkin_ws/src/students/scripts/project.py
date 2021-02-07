#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# Final Project - Robots Moviles
# 
#
import sys
import rospy 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sound_play.msg import SoundRequest
from std_msgs.msg import String

NAME = "ARGUELLES_MACOSAY"

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
    if order==1:
        posx,posy=(0,0)
    elif order==2:
        posx,posy=(6,0)
    elif order==3:
        posx,posy=(5,5)
    elif order==4:
        posx,posy=(3,0)
    elif order==5:
        posx,posy=(8,5)
    dir.pose.position.x=posx
    dir.pose.position.y=posy
    dir.pose.position.w=0
    publishing_dir.publish(dir)
    print("goal:"+ posx+","+posy)
    return

def listening(order):
    order_received=order.data
    if (order_received=="ROBOT GO TO THE ENTRACE"):
        #robot_talk("did you said go to the entrace")
        position(1)
    elif (order_received=="ROBOT GO TO THE BEDROOM"):
        #robot_talk("did you said go to the bedroom")
        position(2)
    elif (order_received=="ROBOT GO TO THE LIVINGROOM"):
        #robot_talk("did you said go to the livingroom")
        position(3)
    elif (order_received=="ROBOT GO TO THE CORRIDOR"):
        #robot_talk("did you said go to the corridor")
        position(4)
    elif (order_received=="ROBOT GO TO THE KITCHEN"):
        #robot_talk("did you said go to the kitchen")
        position(5)
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
