#!/usr/bin/env python

import rospy
import os

from std_msgs.msg import String
from sound_play.msg import SoundRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

NAME = "Torres_Trejo"
pub_speech = None
pub_mvn = None
loop = None


def robot_control(command):

	robot_position = PoseStamped()
	global progress
	progress = False

	if(command == "ROBOT GO TO BATHROOM"):
		print("I am going to Bathroom")
		robot_position.pose.position.x = 8
		robot_position.pose.position.y = 5
		pub_mvn.publish(robot_position)
		
	elif(command == "ROBOT GO TO LIVINGROOM"):
		robot_position.pose.position.x = 3
		robot_position.pose.position.y = 4
		pub_mvn.publish(robot_position)
		print("I am going to Livingroom")

	elif(command == "ROBOT GO TO KITCHEN"):
		print("I am going to Kitchen")
		robot_position.pose.position.x = 3
		robot_position.pose.position.y = 1
		pub_mvn.publish(robot_position)

	elif(command == "ROBOT GO TO HOME"):
		print("I am going to Home")
		robot_position.pose.position.x = 0
		robot_position.pose.position.y = 0
		pub_mvn.publish(robot_position)

	else:
		print("Command Not Found :(")



def callback_goal(msg):
	x = msg.linear.x
	y = msg.linear.y


def callback_voice(msg):
	command = msg.data
	robot_control(command)

"""
def robot_say():
	print(ALERT)
	msg_speech = SoundRequest()
	msg_speech.sound   = -3
	msg_speech.command = 1
	msg_speech.volume  = 1.0
	msg_speech.arg = ALERT
	msg_speech.arg2 = "voice_kal_diphone"
	pub_speech.publish(msg_speech)
"""


def main():
	global pub_mvn, loop
	print("FINAL PROJECT" + NAME)
	rospy.init_node("finalProject")
	rospy.Subscriber('/recognized', String, callback_voice)
	rospy.Subscriber('/cmd_vel', Twist, callback_goal)
	pub_mvn = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 10)
	loop = rospy.Rate(20)
	rospy.spin()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass