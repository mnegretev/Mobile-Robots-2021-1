#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# Final project - THE PLATFORM ROS 
#
# Description:
# This script take the voice entrance and execute commands
# in order to make the robot move to a marked place
#
import sys
import rospy
import tf
import math
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetPlanRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from sound_play.msg import SoundRequest

NAME = "ROLDAN_RIVERA"

#system variables
listener = None
audioPublisher = None
poseStampedPub = None
soundRequest = SoundRequest()

#Places
KITCHEN = [6, 1]
OFFICE = [3, 3]
ENTRANCE = [3, 0]
LIVINGROOM = [8, 5]

#Varaible to know if the robot is moving or not
robotIsMoving = False
actualPosition = [] #[robotX, robotY, robotA]
goalPosition = []
tolerance = 0.5

def robotSay(textToSpeach):
    global soundRequest, audioPublisher
    
    print textToSpeach
    soundRequest.arg = textToSpeach
    audioPublisher.publish(soundRequest)
    

def readVoiceCommand(data):
    global robotIsMoving, KITCHEN, OFFICE, ENTRANCE, LIVINGROOM, goalPosition
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    
    if(robotIsMoving):
        #Say that im busy
        print "Im in: " + str(get_robot_pose)
        robotSay("Sorry, im busy right now")
    else:
        if(data.data == "ROBOT GO TO THE KITCHEN"):
            #set the goal position = KITCHEN
            print "going to the kitchen"
            robotSay("Im going to the kitchen")
            goalPosition = KITCHEN
        elif(data.data == "ROBOT GO TO THE OFFICE"):
            #set the goal position = CORRIDOR
            print "going to the OFFICE"
            robotSay("Im going to the office")
            goalPosition = OFFICE
        elif(data.data == "ROBOT GO TO THE ENTRANCE"):
            #set the goal position = ENTRANCE
            print "going to the ENTRANCE"
            robotSay("Im going to the entrance")
            goalPosition = ENTRANCE
        elif(data.data == "ROBOT GO TO THE LIVINGROOM"):
            #set the goal position = LIVINGROOM
            print "going to the LIVINGROOM"
            robotSay("Im going to the livingroom")
            goalPosition = LIVINGROOM
        
        robotIsMoving = True
        #publish the 2D movement arrow
        poseStamp = PoseStamped()
        pose = Pose()   #Contains a point and a quaternion
            #point
        point = Point()
        point.x = goalPosition[0]
        point.y = goalPosition[1]
            #Quaternion
        quat = Quaternion()
        quat.x = 0
        quat.y = 0
        quat.z = 0
        quat.w = 1

        poseStamp.pose.position = point
        poseStamp.pose.orientation = quat
        poseStampedPub.publish(poseStamp)

            
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
    #initializing
    global listener, tolerance, robotIsMoving, audioPublisher, poseStampedPub, soundRequest
    rospy.init_node("finalPractice")
    print "Final Practice - " + NAME
    listener = tf.TransformListener()

    #Sound requst configuration
    soundRequest.sound = -3
    soundRequest.command = 1
    soundRequest.volume = 1.0
    soundRequest.arg2 = 'voice_kal_diphone'

    # Suscribers and publishers
    rospy.Subscriber("/recognized", String, readVoiceCommand)
    audioPublisher = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    poseStampedPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    while(not rospy.is_shutdown()):
        if(robotIsMoving):
            #Calculating the error between the actual position and goal position
            actualPosition = get_robot_pose(listener)
            globalErrorX = actualPosition[0] - goalPosition[0]
            globalErrory = actualPosition[1] - goalPosition[1]
            globalError = math.sqrt(globalErrorX**2 + globalErrory**2)

            print globalError
            if(globalError < tolerance):
                #Sat that i arrived
                print("I arrived")
                robotSay("I arrived finally sir")
                robotIsMoving = False

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
