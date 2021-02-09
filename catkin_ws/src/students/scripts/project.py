#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# Final Project - Robots Moviles
# 
#
import sys
import rospy
import tf
import math
import numpy

from geometry_msgs.msg import PoseStamped
from sound_play.msg import SoundRequest
from std_msgs.msg import String

NAME = "ARGUELLES_MACOSAY"

confirm=0
actual_pos=1
place=0
pos_goal=0
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
    global pos_goal
    pos_goal=posx,posy
    print("goal:"+ str(pos_goal))
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
                robot_talk("im already going or in the place")
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

def arrived(odom):
    tolerance=0.5
    [robot_act_x,robot_act_y,robot_act_a]= get_robot_pose(listener)
    global pos_goal
    print("pos actual:" +str(robot_act_x)+","+str(robot_act_y))
    print("goal pos:" + str(pos_goal ))
    print("")
    place_arrived=""
    not_arrived=True
    while(not_arrived):
        [robot_act_x,robot_act_y,robot_act_a]= get_robot_pose(listener)
        #print("pos actual:" +str(robot_act_x)+","+str(robot_act_y))
        pos_tolerance_x= numpy.fabs(robot_act_x-pos_goal[0])
        pos_tolerance_y= numpy.fabs(robot_act_y-pos_goal[1])
        #print("diff_de_pos: "+str(pos_tolerance_x)+","+str(pos_tolerance_y))
        if(pos_tolerance_x<=tolerance and pos_tolerance_y<=tolerance):
            not_arrived=False
            if (place==1):
                place_arrived="entrance"
            elif (place==2):
                place_arrived="bedroom"
            elif (place==3):
                place_arrived="livingroom"
            elif (place==4):
                place_arrived="corridor"
            elif (place==5):
                place_arrived="kitchen"
            robot_talk("I have arrived to the " + place_arrived)

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
    global publishing_robot_voice
    global publishing_dir
    global listener

    print ("PROJECT " + NAME)
    rospy.init_node("project")
    publishing_dir=rospy.Publisher('/move_base_simple/goal', PoseStamped,queue_size=10)
    publishing_robot_voice=rospy.Publisher('/robotsound',SoundRequest,queue_size=10)
    rospy.Subscriber("/recognized",String,listening)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, arrived)
    listener = tf.TransformListener()
    rospy.Rate(50)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
