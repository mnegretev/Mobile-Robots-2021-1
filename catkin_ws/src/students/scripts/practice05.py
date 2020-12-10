#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 5 - POSITION CONTROL AND PATH TRACKING
#
# Instructions:
# Write the code necessary to move the robot along a given path.
# Consider a differential base. Max linear and angular speeds
# must be 0.8 and 1.0 respectively.
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

NAME = "MENDOZA_TOLEDO_OSCAR"

pub_cmd_vel = None
loop        = None
listener    = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()

    #
    # TODO:
    # Implement the control law given by:
    #
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
    # where error_a is the angle error and
    # v and w are the linear and angular speeds taken as input signals
    # and v_max, w_max, alpha and beta, are tunning constants.
    # Store the resulting v and w in the Twist message cmd_vel
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #
    w_max = 0.5
    v_max = 0.5
    alpha = 1.0
    beta  = 1.0

    error_a = math.atan2(goal_y - robot_y, goal_x - robot_x) - robot_a

    if error_a > math.pi:
        error_a -= 2*math.pi # Para que regrese a -pi
    elif error_a < -math.pi:
        error_a += 2*math.pi

    cmd_vel.linear.x = v_max * math.exp(-error_a * error_a / alpha)
    cmd_vel.angular.z = w_max * (2 / (1 + math.exp(-error_a / beta)) - 1)

    return cmd_vel

def follow_path(path):
    #
    # TODO:
    # Use the calculate_control function to move the robot along the path.
    # Path is given as a sequence of points [[x0,y0], [x1,y1], ..., [xn,yn]]
    # The publisher for the twist message is already declared as 'pub_cmd_vel'
    # You can use the following steps to perform the path tracking:
    #
    # Set local goal point as the first point of the path
    posicion = 0
    local_goal_x = path[posicion][0]
    local_goal_y = path[posicion][1]
    # Set global goal point as the last point of the path
    global_goal_x = path[len(path)-1][0]
    global_goal_y = path[len(path)-1][1]
    # Get robot position with [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    robot_x, robot_y, robot_a = get_robot_pose(listener)
    # Calculate global error as the magnitude of the vector from robot pose to global goal point
    global_error = math.sqrt(((global_goal_x - robot_x)*(global_goal_x - robot_x)) + ((global_goal_y - robot_y)*(global_goal_y - robot_y)))
    # Calculate local  error as the magnitude of the vector from robot pose to local  goal point
    local_error = math.sqrt(((local_goal_x - robot_x)*(local_goal_x - robot_x)) + ((local_goal_y - robot_y)*(local_goal_y - robot_y)))
    # WHILE global error > tolerance  and not rospy.is_shutdown() #This keeps the program aware of signals such as Ctrl+C
    tolerancia = 0.01
    while posicion < (len(path)-1) and global_error > tolerancia and not rospy.is_shutdown():
        #Calculate control signals v and w and publish the corresponding message
        pub_cmd_vel.publish(calculate_control(robot_x, robot_y, robot_a, local_goal_x, local_goal_y))
        #loop.sleep()  #This is important to avoid an overconsumption of processing time
        loop.sleep()
        #Get robot position
        robot_x, robot_y, robot_a = get_robot_pose(listener)
        #Calculate local error
        local_error = math.sqrt(((local_goal_x - robot_x)*(local_goal_x - robot_x)) + ((local_goal_y - robot_y)*(local_goal_y - robot_y)))
        #If local error is less than 0.3 (you can change this constant)
        if local_error < 0.3:
            #Change local goal point to the next point in the path
            posicion += 1
            local_goal_x = path[posicion][0]
            local_goal_y = path[posicion][1]
        #Calculate global error
        global_error = math.sqrt(((global_goal_x - robot_x)*(global_goal_x - robot_x)) + ((global_goal_y - robot_y)*(global_goal_y - robot_y)))
    print("Ha llegado a su destino")

    # Send zero speeds (otherwise, robot will keep moving after reaching last point)
    cmd_vel = Twist()
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0
    pub_cmd_vel.publish(cmd_vel)

    return

def callback_global_goal(msg):
    print "Calculatin path from robot pose to " + str([msg.pose.position.x, msg.pose.position.y])
    clt_plan_path = rospy.ServiceProxy('/navigation/path_planning/a_star_search', GetPlan)
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    req = GetPlanRequest()
    req.start.pose.position.x = robot_x
    req.start.pose.position.y = robot_y
    req.goal.pose.position.x  = msg.pose.position.x
    req.goal.pose.position.y  = msg.pose.position.y
    path = clt_plan_path(req).plan
    print "Following path with " + str(len(path.poses)) + " points..."
    path =[[p.pose.position.x, p.pose.position.y] for p in path.poses]
    follow_path(path)
    print "Global goal point reached"

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
    return None

def main():
    global pub_cmd_vel, loop, listener
    print "PRACTICE 05 - " + NAME
    rospy.init_node("practice05")
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    loop = rospy.Rate(20)
    listener = tf.TransformListener()
    listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(5.0))
    rospy.wait_for_service('/navigation/path_planning/a_star_search')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
