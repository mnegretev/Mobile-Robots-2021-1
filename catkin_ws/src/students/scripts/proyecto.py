#!/usr/bin/env python

import numpy as np
import glob
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

import roslib; roslib.load_manifest('sound_play')
from sound_play.libsoundplay import SoundClient 
from sound_play.msg import SoundRequest

#pub_coord = None
pub_pose=None



def to_order(word):

    global caminando
    global ordenado

    localization = PoseStamped()
    pub_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    
    if (caminando==False):
        if word=='GO TO KITCHEN':
            localization.pose.position.x = 3
            localization.pose.position.y = 0
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Yendo a cocina: coordenadas 1,1'
        if word=='GO TO LIVINGROOM':
            localization.pose.position.x = 3
            localization.pose.position.y = 6
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Yendo a sala: coordenadas 3,6'
        if word=='GO TO BEDROOM':
            localization.pose.position.x = 8
            localization.pose.position.y = 0
            localization.pose.orientation.w = 1
            pub_pose.publish(localization)
            print 'Yendo a recamara: coordenadas 8,0'
        ordenado=True
    if (caminando==True):
        print('orita no joven')

    return 0

def callback_function(msg):
    orden = msg.data
    to_order(orden)

def decir(oracion):
    soundhandle = SoundClient()     
    s = " ".join(oracion)
    print 'Saying: %s'%s
    soundhandle.say(s)
    rospy.sleep(1)

def callback_checar(msg):
    
    global caminando
    global ordenado

    if(msg.linear.x!=0):
        caminando = True
        #print('####################################################################################################recibido') 

    if(msg.linear.x==0 and ordenado):
        ordenado =False
        caminando =False
        oracion="I have arrived"
        print(oracion)
        decir(oracion)
    return


def main():
    global caminando
    global ordenado

    ordenado=False
    caminando=False

    print(ordenado, caminando,'########################################################################')

    #rospy.init_node("proyecto_jmf")
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

