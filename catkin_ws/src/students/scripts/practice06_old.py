#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-1
# PRACTICE 6 - OBSTACLE AVOIDANCE BY POTENTIAL FIELDS
#
# Instructions:
# Complete the code to implement obstacle avoidance by potential fields
# using the attractive and repulsive fields technique.
# Tune the constants alpha and beta to get a smooth movement. 
#

import rospy
import tf
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

NAME = "ANTONIO_GARCIA"
listener    = None
pub_cmd_vel = None
pub_markers = None

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
    
    alpha = 0.1
    beta  = 0.9
    v_max = 1
    w_max = 0.5
    error_a = 0.001

    goal_a = math.atan2(goal_y - robot_y, goal_x - robot_x)
    error_a = goal_a - robot_a

    if error_a > math.pi:
        error_a -= 2*math.pi

    if error_a < -math.pi:
        error_a += 2*math.pi

    if robot_x == goal_x and robot_y == goal_y:
    	v_max = 0
        w_max = 0

    v = v_max*math.exp(-error_a*error_a/alpha)
    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    cmd_vel.linear.x = v
    cmd_vel.angular.z = w
    return cmd_vel

def attraction_force(robot_x, robot_y, goal_x, goal_y):
    #
    # TODO:
    # Calculate the attraction force, given the robot and goal positions.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting attraction force w.r.t. map.
    #
    force_x, force_y = [0, 0]
    alpha = 2.5
    modulo = math.sqrt(math.pow(robot_x - goal_x,2) + math.pow(robot_y - goal_y,2))
    force_x = alpha * ((robot_x - goal_x)/modulo)
    force_y = alpha * ((robot_y - goal_y)/modulo)
    return [force_x, force_y]

def rejection_force(robot_x, robot_y, robot_a, laser_readings):
    #
    # TODO:
    # Calculate the total rejection force given by the average
    # of the rejection forces caused by each laser reading.
    # laser_readings is an array where each element is a tuple [distance, angle]
    # both measured w.r.t. robot's frame.
    # See lecture notes for equations to calculate rejection forces.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting rejection force w.r.t. map.
    #
    force_x, force_y = [0, 0]
    N = len(laser_readings)
    beta = 3
    d = 0.5
    if (N > 0):
	for i in range(N - 1):
	    distance, angle = laser_readings[i]
	    if d <= distance:
	        force_x += beta * math.sqrt(1/d - 1/distance) * (math.cos(angle + robot_a))
	        force_y += beta * math.sqrt(1/d - 1/distance) * (math.sin(angle + robot_a))
        force_x = force_x/N 
        force_y = force_y/N
    #print(force_x, " ",force_y)
    return [force_x, force_y]

def callback_pot_fields_goal(msg):
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    print "Moving to goal point " + str([goal_x, goal_y]) + " by potential fields"    
    loop = rospy.Rate(20)
    global laser_readings

    #
    # TODO:
    # Move the robot towards goal point using potential fields.
    # Remember goal point is a local minimun in the potential field, thus,
    # it can be reached by the gradient descend algorithm.
    # Sum of attraction and rejection forces is the gradient of the potential field,
    # then, you can reach the goal point with the following pseudocode:
    #
    # Set constant epsilon (0.5 is a good start)
    # Set tolerance  (0.1 is a good start)
    # Get robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
    # Calculate distance to goal as math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    # WHILE distance_to_goal_point > tolerance and not rospy.is_shutdown():
    #     Calculate attraction force Fa by calling [fax, fay] = attraction_force(robot_x, robot_y, goal_x, goal_y)
    #     Calculate rejection  force Fr by calling [frx, fry] = rejection_force (robot_x, robot_y, robot_a, laser_readings)
    #     Calculate resulting  force F = Fa + Fr
    #     Calculate next local goal point P = [px, py] = Pr - epsilon*F
    #
    #     Calculate control signals by calling msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
    #     Send the control signals to mobile base by calling pub_cmd_vel.publish(msg_cmd_vel)
    #     Call draw_force_markers(robot_x, robot_y, afx, afy, rfx, rfy, fx, fy, pub_markers)  to draw all forces
    #
    #     Wait a little bit of time by calling loop.sleep()
    #     Update robot position by calling robot_x, robot_y, robot_a = get_robot_pose(listener)
    #     Recalculate distance to goal position
    #  Publish a zero speed (to stop robot after reaching goal point)
    cmd_vel = Twist()
    epsilon = 0.5
    tolerance = 0.1
    robot_x, robot_y, robot_a = get_robot_pose(listener)
    distance_to_goal_point = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
	
    while distance_to_goal_point > tolerance and not rospy.is_shutdown():
        [fax, fay] = attraction_force(robot_x, robot_y, goal_x, goal_y)
	[frx, fry] = rejection_force (robot_x, robot_y, robot_a, laser_readings)
	fx = fax + frx
	fy = fay + fry
	px = robot_x - epsilon * fx
	py = robot_y - epsilon * fy 
	msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
	pub_cmd_vel.publish(msg_cmd_vel)
	draw_force_markers(robot_x, robot_y, fax, fay, frx, fry, fx, fy, pub_markers)
	loop.sleep()
	robot_x, robot_y, robot_a = get_robot_pose(listener)
	distance_to_goal_point = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        #print(distance_to_goal_point)
    pub_cmd_vel.publish(calculate_control(0, 0, 0, 0, 0))
    print("Goal point reached")

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

def callback_scan(msg):
    global laser_readings
    laser_readings = [[0,0] for i in range(len(msg.ranges))]
    for i in range(len(msg.ranges)):
        laser_readings[i] = [msg.ranges[i], msg.angle_min + i*msg.angle_increment]

def draw_force_markers(robot_x, robot_y, attr_x, attr_y, rej_x, rej_y, res_x, res_y, pub_markers):
    pub_markers.publish(get_force_marker(robot_x, robot_y, attr_x, attr_y, [0,0,1,1]  , 0))
    pub_markers.publish(get_force_marker(robot_x, robot_y, rej_x,  rej_y,  [1,0,0,1]  , 1))
    pub_markers.publish(get_force_marker(robot_x, robot_y, res_x,  res_y,  [0,0.6,0,1], 2))

def get_force_marker(robot_x, robot_y, force_x, force_y, color, id):
    mrk = Marker()
    mrk.header.frame_id = "map"
    mrk.header.stamp    = rospy.Time.now()
    mrk.ns = "pot_fields"
    mrk.id = id
    mrk.type   = Marker.ARROW
    mrk.action = Marker.ADD
    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
    mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
    mrk.points.append(Point())
    mrk.points.append(Point())
    mrk.points[0].x, mrk.points[0].y = robot_x,  robot_y
    mrk.points[1].x, mrk.points[1].y = robot_x - force_x, robot_y - force_y
    return mrk

def main():
    global listener, pub_cmd_vel, pub_markers
    print "PRACTICE 06 - " + NAME
    rospy.init_node("practice06")
    rospy.Subscriber("/scan", LaserScan, callback_scan)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_pot_fields_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist,  queue_size=10)
    pub_markers = rospy.Publisher('/navigation/pot_field_markers', Marker, queue_size=10)
    listener = tf.TransformListener()
    #listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(5.0))
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
