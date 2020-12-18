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

NAME = "MARTINEZ FADUL JESUS"
listener    = None
pub_cmd_vel = None
pub_markers = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):

    cmd_vel = Twist()
    
    v_max=1
    w_max=0.5
    alpha=0.01
    beta=0.1

    th_g=math.atan2(goal_y-robot_y,goal_x-robot_x)
    error_a=th_g-robot_a


    if error_a > math.pi:
        error_a=error_a-(2*math.pi)

    if error_a < -math.pi:
        error_a=error_a+(2*math.pi)

    v = v_max*math.exp(-error_a*error_a/alpha)
    w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)

    cmd_vel.linear.x=v
    cmd_vel.angular.z=w
    
    return cmd_vel

def attraction_force(robot_x, robot_y, goal_x, goal_y):
    #
    # TODO:
    # Calculate the attraction force, given the robot and goal positions.
    # Return a tuple of the form [force_x, force_y]
    # where force_x and force_y are the X and Y components
    # of the resulting attraction force w.r.t. map.
    #

    alpha=1
    mag=((robot_x - goal_x)**2+(robot_y - goal_y)**2)**0.5

    force_x= alpha*((robot_x - goal_x)/mag)
    force_y= alpha*((robot_y - goal_y)/mag)
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
    beta=4  

    Fx=0
    Fy=0

    for c in range(len(laser_readings)):
        dis,ang=laser_readings[c]

        if dis<1:
            Fx=Fx+beta*math.sqrt(1/dis -1/1)*math.cos(ang+robot_a)
            Fy=Fy+beta*math.sqrt(1/dis -1/1)**0.5*math.sin(ang+robot_a)
        if dis>=1:
            Fx=Fx+0;
            Fy=Fy+0;

    force_x=Fx/len(laser_readings)
    force_y=Fy/len(laser_readings)

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
    epsilon=0.5

    [r_x,r_y,r_a]=get_robot_pose(listener)
    d_r=math.sqrt((goal_x - r_x)**2 + (goal_y - r_y)**2)

    while d_r > 0.1 and not rospy.is_shutdown():

        [fax, fay] = attraction_force(r_x, r_y, goal_x, goal_y)
        [frx, fry] = rejection_force (r_x, r_y, r_a, laser_readings)
 
        Fx=fax+frx 
        Fy=fay+fry

        px=r_x-epsilon*Fx
        py=r_y-epsilon*Fy
        
        msg_cmd_vel = calculate_control(r_x, r_y, r_a, px, py)
        pub_cmd_vel.publish(msg_cmd_vel)
        draw_force_markers(r_x, r_y, fax, fay, frx, fry, Fx, Fy, pub_markers)

        #print(Fx)
        #print(Fy)

        loop.sleep()
        [r_x,r_y,r_a]=get_robot_pose(listener)
        d_r=math.sqrt((goal_x - r_x)**2 + (goal_y - r_y)**2)

    msg_cmd_vel.linear.x=0.0
    msg_cmd_vel.angular.z=0.0
    pub_cmd_vel.publish(msg_cmd_vel)
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
    
