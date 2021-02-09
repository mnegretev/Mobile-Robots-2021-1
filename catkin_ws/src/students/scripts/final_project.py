#!/usr/bin/python

import rospy

from std_msgs.msg import String

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from sound_play.msg import SoundRequest

NAME = "ANTONIO_GARCIA"
ROBOT_PHRASE1 = "I arrived"
ROBOT_PHRASE2 = "busy"
#loop = None
pub_pose = None
pub_sound = None
voice = None
mov_flag = False
repeat = True

def state_machine(message):
    global mov_flag, pub_pose
    pos = PoseStamped()
    if(mov_flag==True):
        print 'Robot en movimiento'
    elif(message == 'ENTRANCE'):
        mov_flag = True
        pos.pose.position.x = 0
        pos.pose.position.y = 0
        pos.pose.orientation.w = 1
        pub_pose.publish(pos)
        print 'En direccion a la entrada'
    elif(message == 'KITCHEN'):
        mov_flag = True
        pos.pose.position.x = 4
        pos.pose.position.y = 0
        pos.pose.orientation.w = 1
        pub_pose.publish(pos)
        print 'En direcciona a la cocina'
    elif(message == 'BEDROOM'):
        mov_flag = True
        pos.pose.position.x = 8
        pos.pose.position.y = 2
        pos.pose.orientation.w = 1
        pub_pose.publish(pos)
        print 'En direccion al cuarto'
    else:
        print 'El mensaje: '+message+' no corresponde con ningun tipo de orden.'

def callback_pose(msg):
    global voice, pub_sound, mov_flag, repeat
    if(msg.linear.x == 0 and msg.angular.x == 0):
        mov_flag = False
        print 'El robot llego a su destino'
        voice.arg = ROBOT_PHRASE1
        pub_sound.publish(voice)
        repeat = True
    elif(repeat == True):
        repeat = False
        voice.arg = ROBOT_PHRASE2
	pub_sound.publish(voice)

def callback_msg(msg):
    message = msg.data
    state_machine(message)

def main():
    global pub_pose, pub_sound, voice#,loop
    print "Proyecto final - " + NAME
    voice = SoundRequest()
    voice.sound = 1 
    voice.command = 1
    voice.volume= 1.0
    pub_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pub_sound = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    rospy.init_node("final_project")
    rospy.Subscriber('/cmd_vel', Twist, callback_pose)
    rospy.Subscriber('/recognized', String, callback_msg)
    #loop = rospy.Rate(20) # 20Hz
    rospy.wait_for_service('/static_map')
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
