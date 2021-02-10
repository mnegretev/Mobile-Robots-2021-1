#!/usr/bin/env python
#

import sys
import rospy
import tf
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sound_play.msg import SoundRequest

NAME = "rangel_navarro"


global dist
pub_voice = None
pub_goal = None
goal = None 
tol = None
state = "idle"
loop = None
global_g = None
sub_voice = None
sub_gen = None


def catch_the_voice(data):
	global goal, loop, dist

	place = None 
	rospy.loginfo(rospy.get_caller_id() + "Reconocimiento de voz: %s", data.data)
	goal = PoseStamped()
        word_extend = data.data
	word = word_extend.split()

	print 'seleccion del estado'
	if(word[-1] == 'KITCHEN'):
		goal.pose.position.x = 8.0
		goal.pose.position.y = 5.0
		goal.pose.position.z = 0.0
		place = 'Loc_3'

	elif (word[-1] == 'LIVINGROOM'):
		goal.pose.position.x = 3.0
		goal.pose.position.y = 0.0
		goal.pose.position.z = 0.0
		place = 'Loc_2'

	elif (word[-1] == 'BEDROOM'):
		goal.pose.position.x = 3.0
		goal.pose.position.y = 4.0
		goal.pose.position.z = 0.0
		place = 'loc_1'

	else:
		goal.pose.position.x = 0.0
		goal.pose.position.y = 0.0
		goal.pose.position.z = 0.0
		place = "inicio"
		state = False 
		
	pub_goal.publish(goal)
	loop.sleep()

def genera_voz(place):
	pub_voice.publish(-3,1,1.0, place)
	
def get_robot_pose(listener):
    try:
        (trans, rot) = listener.lookupTransform('map','base_link', rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
        robot_a = 2*math.atan2(rot[2], rot[3])
        if robot_a > math.pi:
            robot_a -= 2*math.pi
        return robot_x, robot_y, robot_a
    except:
        pass
    return [0,0,0]

def ignored_noise(): 
	[x_r,y_r,a_r] = get_robot_pose(listener)
	xg = goal.pose.position.x 
	yg = goal.pose.position.y
	dist = math.sqrt((xg-x_r)**2 + (yg-y_r)**2)
	if dis < 1.5: 
	   genera_voz(place)
	
def main():
	global pub_voice, pub_goal, loop, state, sub_voice, sub_gen
	rospy.init_node("proyecto")
	loop = rospy.Rate(20)
	pub_voice = rospy.Publisher('/robotsound',SoundRequest,queue_size=10)
	pub_goal = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)
	pub_gen = rospy.Publisher('/robotsound',SoundRequest,queue_size=10)

	listener = tf.TransformListener()
	
	sub_voice = rospy.Subscriber('/recognized',String,catch_the_voice)
	while not rospy.is_shutdown():
        	loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


