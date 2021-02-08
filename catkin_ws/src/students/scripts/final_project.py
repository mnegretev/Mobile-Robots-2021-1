#!/usr/bin/python

import rospy

from std_msgs.msg import String

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from sound_play.msg import SoundRequest

NAME = "ANTONIO_GARCIA"
ROBOT_PHRASE1 = "I have arrived"
ROBOT_PHRASE2 = "I am busy"

def state_machine(message):
    pos = PoseStamped()
    if(mov_flag==True):
        print 'Robot en movimiento'
    elif(message == 'ROBOT GO TO THE ENTRANCE'):
        mov_flag = True
        pos.pose.x = 0
        pos.pose.y = 0
        pos.pose.w = 0
        pub_pose.publish(pos)
        print 'En direccion a la entrada'
    elif(message == 'ROBOT GO TO THE KITCHEN'):
        mov_flag = True
        pos.pose.x = 4
        pos.pose.y = 0
        pos.pose.w = 1
        pub_pose.publish(pos)
        print 'En direcciona a la cocina'
    elif(message == 'ROBOT GO TO THE BEDROOM'):
        mov_flag = True
        pos.pose.x = 8
        pos.pose.y = 2
        pos.pose.w = 1
        pub_pose.publish(pos)
        print 'En direccion al cuarto'
    else:
        print 'El mensaje: '+message+' no corresponde con ningun tipo de orden.'

def callback_pose(msg):
    #voice.sound = 1 
    #voice.command = 1
    #voice.volume= 1.0
    if(msg.linear.x == 0 and msg.angular.x == 0):
        mov_flag = False
        print 'El robot llego a su destino'
        voice.arg = ROBOT_PHRASE1
        pub_sound.publish(voice)
    else:
        voice.arg = ROBOT_PHRASE2
	pub_sound.publish(voice)

def callback_msg(msg):
    message = msg.data
    state_machine(message)

def main():
    global loop, pub_pose, pub_sound, mov_flag, voice
    print "Proyecto final - " + NAME
    mov_flag = False
    pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=10)
    pub_sound = rospy.Publisher('/soundRequest', SoundRequest, queue_size=10)
    voice = SoundRequest()
    rospy.init_node("final_project")
    rospy.Subscriber('/cmd_vel', Twist, callback_pose)
    rospy.Subscriber('/chatter', String, callback_msg)
    #loop = rospy.Rate(20) # 20Hz
    rospy.wait_for_service('/static_map')
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
